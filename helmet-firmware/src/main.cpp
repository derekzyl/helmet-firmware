#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include "pins.h"
#include "config.h"
#include "types.h"
#include "crypto_utils.h"
#include "alcohol_processor.h"
#include "collision_detector.h"

// ==============================================================================
// GLOBAL SYSTEM INSTANCES & STATE
// ==============================================================================

Preferences prefs;
WebSocketsClient wsClient;
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);   // UART2: GPIO 16 (RX), GPIO 17 (TX)
HardwareSerial simSerial(1);   // UART1: GPIO 26 (RX), GPIO 25 (TX)

AlcoholProcessor alcohol;
CollisionDetector collision;

HelmetTelemetryFrame currentFrame;
IgnitionState lastIgnitionState = IgnitionState::LOCKED;
LockReason lastLockReason = LockReason::NO_LINK;

// Configuration & Credentials (loaded from NVS)
String deviceId = DEFAULT_DEVICE_ID;
String pairedIgnitionId = DEFAULT_PAIRED_IGNITION_ID;
String pairingPsk = DEFAULT_PAIRING_PSK;
String wifiSsid = "SmartHelmet_AP";
String wifiPass = "helmet1234";
String ignitionHost = "192.168.4.1"; // default fallback SoftAP gateway
int ignitionPort = LOCAL_SERVER_PORT;
String cloudHost = CLOUD_BACKEND_HOST;
int cloudPort = CLOUD_BACKEND_PORT;

// Emergency Contacts (cached in NVS)
String emergencyContact1 = "+2348000000001";
String emergencyContact2 = "+2348000000002";

// Non-blocking Timers
uint32_t lastHeartbeatMs = 0;
uint32_t lastCloudTelemetryMs = 0;
uint32_t lastSensorPollMs = 0;
uint32_t lastBlinkMs = 0;
uint32_t irWornStartMs = 0;
uint32_t seqCounter = 1;

// Button Debounce Timers
uint32_t btnAPressStartMs = 0;
uint32_t btnBPressStartMs = 0;
bool mpuPresent = false;
bool simPresent = false;
bool simulationMode = false;

// Offline Telemetry Queue (Ring Buffer of 16 frames)
#define OFFLINE_QUEUE_CAPACITY 16
String offlineQueue[OFFLINE_QUEUE_CAPACITY];
uint8_t queueHead = 0;
uint8_t queueTail = 0;
uint8_t queueCount = 0;

void enqueueOfflineTelemetry(const String& payload) {
    if (queueCount < OFFLINE_QUEUE_CAPACITY) {
        offlineQueue[queueTail] = payload;
        queueTail = (queueTail + 1) % OFFLINE_QUEUE_CAPACITY;
        queueCount++;
    } else {
        // Drop oldest to accept newest
        offlineQueue[queueHead] = payload;
        queueHead = (queueHead + 1) % OFFLINE_QUEUE_CAPACITY;
        queueTail = (queueTail + 1) % OFFLINE_QUEUE_CAPACITY;
    }
}

// ==============================================================================
// BUZZER & LED ACTUATION
// ==============================================================================

void setLeds(bool green, bool yellow, bool red) {
    digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW, yellow ? HIGH : LOW);
    digitalWrite(PIN_LED_RED, red ? HIGH : LOW);
}

void beepBuzzer(uint16_t duration_ms) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(duration_ms);
    digitalWrite(PIN_BUZZER, LOW);
}

// ==============================================================================
// SIM800L GSM/GPRS AT-COMMAND DRIVER
// ==============================================================================

void initSim800L() {
    pinMode(PIN_SIM_RST, OUTPUT);
    digitalWrite(PIN_SIM_RST, HIGH);
    delay(100);
    digitalWrite(PIN_SIM_RST, LOW);
    delay(200);
    digitalWrite(PIN_SIM_RST, HIGH);
    delay(1500);

    simSerial.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
    simSerial.println("AT");
    delay(200);
    if (simSerial.available()) {
        String resp = simSerial.readString();
        if (resp.indexOf("OK") >= 0) {
            simPresent = true;
            Serial.println("[SIM800L] Modem detected and responding OK");
            simSerial.println("AT+CMGF=1"); // SMS text mode
            delay(100);
            simSerial.println("AT+CNMI=2,2,0,0,0"); // direct SMS output
        }
    }
    if (!simPresent) {
        Serial.println("[SIM800L] No response, running in software fallback mode");
    }
}

void sendEmergencySms(const String& phone, double lat, double lng, float speed, float impact_g) {
    if (phone.length() == 0) return;
    Serial.printf("[SIM800L] Dispatching Emergency SMS to %s\n", phone.c_str());

    String msg = "EMERGENCY ALERT: Smart Helmet detected a severe collision!\n";
    msg += "Impact: " + String(impact_g, 1) + "G\n";
    msg += "Speed: " + String(speed, 1) + " km/h\n";
    msg += "Location: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lng, 6);

    if (simPresent) {
        simSerial.println("AT+CMGS=\"" + phone + "\"");
        delay(300);
        simSerial.print(msg);
        delay(100);
        simSerial.write(26); // Ctrl+Z
        delay(2000);
    } else {
        Serial.println("[SIM800L-SIM] SMS Simulation: " + msg);
    }
}

// ==============================================================================
// WEBSOCKET LOCAL PROTOCOL HANDSHAKE & HEARTBEAT
// ==============================================================================

void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            currentFrame.local_link_connected = false;
            lastIgnitionState = IgnitionState::LOCKED;
            lastLockReason = LockReason::NO_LINK;
            Serial.println("[WS-LOCAL] Disconnected from Ignition Unit");
            break;

        case WStype_CONNECTED:
            currentFrame.local_link_connected = true;
            Serial.println("[WS-LOCAL] Connected to Ignition Unit!");
            break;

        case WStype_TEXT: {
            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, payload, length);
            if (err) {
                Serial.println("[WS-LOCAL] Invalid JSON received");
                return;
            }

            const char* msgType = doc["type"] | "";
            if (strcmp(msgType, "HEARTBEAT_ACK") == 0) {
                const char* stateStr = doc["data"]["state"] | "LOCKED";
                const char* reasonStr = doc["data"]["lock_reason"] | "NONE";

                if (strcmp(stateStr, "UNLOCKED") == 0) {
                    lastIgnitionState = IgnitionState::UNLOCKED;
                } else if (strcmp(stateStr, "RUNNING_LATCHED") == 0) {
                    lastIgnitionState = IgnitionState::RUNNING_LATCHED;
                } else if (strcmp(stateStr, "FAULT") == 0) {
                    lastIgnitionState = IgnitionState::FAULT;
                } else {
                    lastIgnitionState = IgnitionState::LOCKED;
                }

                if (strcmp(reasonStr, "NO_HELMET") == 0) lastLockReason = LockReason::NO_HELMET;
                else if (strcmp(reasonStr, "ALCOHOL_DETECTED") == 0) lastLockReason = LockReason::ALCOHOL_DETECTED;
                else if (strcmp(reasonStr, "WARMING_UP") == 0) lastLockReason = LockReason::WARMING_UP;
                else if (strcmp(reasonStr, "CRASH_EVENT") == 0) lastLockReason = LockReason::CRASH_EVENT;
                else if (strcmp(reasonStr, "LOW_BATTERY") == 0) lastLockReason = LockReason::LOW_BATTERY;
                else lastLockReason = LockReason::NONE;
            }
            break;
        }

        default:
            break;
    }
}

void sendLocalHeartbeat() {
    if (!currentFrame.local_link_connected) return;

    StaticJsonDocument<768> doc;
    doc["ver"] = 1;
    doc["type"] = "HEARTBEAT";
    doc["dev"] = deviceId;
    doc["seq"] = seqCounter++;
    doc["ts"] = (uint32_t)(millis() / 1000);

    JsonObject data = doc.createNestedObject("data");
    data["worn"] = currentFrame.helmet_worn;
    data["worn_ms"] = currentFrame.worn_duration_ms;
    data["bac"] = serialized(String(currentFrame.bac_level, 3));
    data["mq3_raw"] = currentFrame.mq3_raw;
    data["mq3_warm"] = currentFrame.mq3_warmed_up;
    data["impact"] = currentFrame.impact_flag;
    data["tilt_deg"] = serialized(String(currentFrame.tilt_deg, 1));
    data["batt_pct"] = currentFrame.battery_pct;
    data["batt_mv"] = currentFrame.battery_mv;
    data["gps_fix"] = currentFrame.gps_fixed;
    data["speed"] = serialized(String(currentFrame.speed_kmh, 1));

    String dataString;
    serializeJson(data, dataString);

    // Canonical sig payload: ver:type:dev:seq:ts:canonical_data_string
    String sigPayload = String(doc["ver"].as<int>()) + ":" +
                        doc["type"].as<String>() + ":" +
                        doc["dev"].as<String>() + ":" +
                        String(doc["seq"].as<uint32_t>()) + ":" +
                        String(doc["ts"].as<uint32_t>()) + ":" +
                        dataString;

    std::string hmacSig = crypto::hmac_sha256_hex(pairingPsk.c_str(), sigPayload.c_str());
    doc["sig"] = hmacSig.c_str();

    String output;
    serializeJson(doc, output);
    wsClient.sendTXT(output);
}

// ==============================================================================
// CLOUD TELEMETRY REST INGEST (FASTAPI)
// ==============================================================================

void postCloudTelemetry() {
    if (WiFi.status() != WL_CONNECTED) {
        // Enqueue to offline buffer
        StaticJsonDocument<512> doc;
        doc["device_id"] = deviceId;
        doc["helmet_worn"] = currentFrame.helmet_worn;
        doc["bac_level"] = currentFrame.bac_level;
        doc["latitude"] = currentFrame.latitude;
        doc["longitude"] = currentFrame.longitude;
        doc["speed_kmh"] = currentFrame.speed_kmh;
        doc["battery_percentage"] = currentFrame.battery_pct;
        doc["ignition_status"] = ignition_state_to_str(lastIgnitionState);
        String json;
        serializeJson(doc, json);
        enqueueOfflineTelemetry(json);
        return;
    }

    HTTPClient http;
    String url = "http://" + cloudHost + ":" + String(cloudPort) + CLOUD_TELEMETRY_ENDPOINT;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    // Flush any pending queued telemetry first
    while (queueCount > 0) {
        String queuedPayload = offlineQueue[queueHead];
        queueHead = (queueHead + 1) % OFFLINE_QUEUE_CAPACITY;
        queueCount--;
        http.POST(queuedPayload);
    }

    StaticJsonDocument<512> doc;
    doc["device_id"] = deviceId;
    doc["helmet_worn"] = currentFrame.helmet_worn;
    doc["bac_level"] = currentFrame.bac_level;
    doc["alcohol_detected"] = alcohol.is_over_threshold();
    doc["latitude"] = currentFrame.latitude;
    doc["longitude"] = currentFrame.longitude;
    doc["speed_kmh"] = currentFrame.speed_kmh;
    doc["battery_percentage"] = currentFrame.battery_pct;
    doc["battery_voltage"] = currentFrame.battery_mv / 1000.0f;
    doc["impact_g"] = currentFrame.impact_g;
    doc["tilt_degrees"] = currentFrame.tilt_deg;
    doc["ignition_status"] = ignition_state_to_str(lastIgnitionState);
    doc["lock_reason"] = lock_reason_to_str(lastLockReason);

    String payload;
    serializeJson(doc, payload);

    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0) {
        // Telemetry delivered successfully
    } else {
        enqueueOfflineTelemetry(payload);
    }
    http.end();
}

void postCloudEmergencyEvent(const char* eventType, const char* desc) {
    if (WiFi.status() != WL_CONNECTED) return;

    HTTPClient http;
    String url = "http://" + cloudHost + ":" + String(cloudPort) + CLOUD_EVENTS_ENDPOINT;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<384> doc;
    doc["device_id"] = deviceId;
    doc["event_type"] = eventType;
    doc["severity"] = "CRITICAL";
    doc["description"] = desc;
    doc["latitude"] = currentFrame.latitude;
    doc["longitude"] = currentFrame.longitude;

    String payload;
    serializeJson(doc, payload);
    http.POST(payload);
    http.end();
}

// ==============================================================================
// SENSOR ACQUISITION & PROCESSING
// ==============================================================================

void pollSensors() {
    uint32_t now = millis();

    // 1. FC-51 Infrared Proximity Sensor (Active-LOW when helmet on head)
    bool rawWorn = (digitalRead(PIN_HELMET_IR) == LOW);
    if (simulationMode) rawWorn = true;

    if (rawWorn) {
        if (irWornStartMs == 0) irWornStartMs = now;
        currentFrame.worn_duration_ms = now - irWornStartMs;
        // Debounce: must be continuously worn for at least 1000ms
        currentFrame.helmet_worn = (currentFrame.worn_duration_ms >= 1000);
    } else {
        irWornStartMs = 0;
        currentFrame.worn_duration_ms = 0;
        currentFrame.helmet_worn = false;
    }

    // 2. MQ-3 Alcohol Sensor on ADC1 (GPIO34)
    uint16_t rawAdc = analogRead(PIN_MQ3_AO);
    if (simulationMode && rawAdc < 300) rawAdc = 420; // clean air baseline
    currentFrame.mq3_raw = rawAdc;

    alcohol.update(rawAdc, now, currentFrame.helmet_worn);
    currentFrame.bac_level = alcohol.get_bac();
    currentFrame.mq3_warmed_up = !alcohol.is_warming_up();

    // 3. Battery Voltage Sense on ADC1 (GPIO35) with 100k/100k divider
    uint16_t rawBatt = analogRead(PIN_BATTERY_SENSE);
    float vBattAdc = (rawBatt / 4095.0f) * 3.3f;
    float vBattActual = vBattAdc * 2.0f; // 100k/100k divider multiplier
    if (vBattActual < 2.5f) vBattActual = 4.05f; // bench / USB default
    currentFrame.battery_mv = (uint16_t)(vBattActual * 1000.0f);
    
    // Li-ion percentage curve approximation (3.4V = 0%, 4.2V = 100%)
    float pct = ((vBattActual - 3.4f) / (4.2f - 3.4f)) * 100.0f;
    currentFrame.battery_pct = (uint8_t)constrain((int)pct, 0, 100);

    // 4. MPU-6050 6-Axis Motion & Crash Detection
    float ax = 0.0f, ay = 0.0f, az = 1.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    if (mpuPresent) {
        int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
        mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);
        ax = rawAx / 16384.0f;
        ay = rawAy / 16384.0f;
        az = rawAz / 16384.0f;
        gx = rawGx / 131.0f;
        gy = rawGy / 131.0f;
        gz = rawGz / 131.0f;
    }

    collision.update(ax, ay, az, gx, gy, gz, now);
    currentFrame.impact_g = collision.get_peak_impact_g();
    currentFrame.tilt_deg = collision.get_tilt_deg();
    currentFrame.impact_flag = collision.is_confirmed_crash();

    if (collision.is_pre_alert()) {
        // Pre-alert countdown active! Buzzer beeps urgently
        if ((now / 250) % 2 == 0) digitalWrite(PIN_BUZZER, HIGH);
        else digitalWrite(PIN_BUZZER, LOW);
        setLeds(false, false, true); // Red LED blinking
    } else if (collision.is_confirmed_crash()) {
        digitalWrite(PIN_BUZZER, HIGH);
        setLeds(false, false, true);
        // Dispatch Emergency SMS via SIM800L and Cloud Event
        static bool emergencySent = false;
        if (!emergencySent) {
            emergencySent = true;
            sendEmergencySms(emergencyContact1, currentFrame.latitude, currentFrame.longitude, currentFrame.speed_kmh, currentFrame.impact_g);
            sendEmergencySms(emergencyContact2, currentFrame.latitude, currentFrame.longitude, currentFrame.speed_kmh, currentFrame.impact_g);
            postCloudEmergencyEvent("COLLISION", "Severe collision detected with rider fall");
        }
    }

    // 5. GPS UART Feed
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid()) {
        currentFrame.gps_fixed = true;
        currentFrame.latitude = gps.location.lat();
        currentFrame.longitude = gps.location.lng();
        currentFrame.speed_kmh = gps.speed.kmph();
        currentFrame.hdop = gps.hdop.hdop();
    } else if (simulationMode) {
        currentFrame.gps_fixed = true;
        currentFrame.latitude = 6.5244 + 0.001 * sin(now / 10000.0);
        currentFrame.longitude = 3.3792 + 0.001 * cos(now / 10000.0);
        currentFrame.speed_kmh = 32.5f;
    }
}

// ==============================================================================
// BUTTON INPUT HANDLING
// ==============================================================================

void handleButtons() {
    uint32_t now = millis();

    // Button A: SOS Cancel / False Alarm Reset
    if (digitalRead(PIN_BUTTON_A) == LOW) {
        if (btnAPressStartMs == 0) btnAPressStartMs = now;
    } else {
        if (btnAPressStartMs > 0) {
            uint32_t duration = now - btnAPressStartMs;
            if (duration >= 50 && duration < 3000) {
                // Short press: cancel pre-alert alarm
                if (collision.is_pre_alert()) {
                    collision.cancel_alarm();
                    digitalWrite(PIN_BUZZER, LOW);
                    Serial.println("[BUTTON-A] False alarm cancelled by rider");
                    beepBuzzer(100);
                }
            }
            btnAPressStartMs = 0;
        }
    }

    // Button B: Hold 5s for BLE Provisioning Mode
    if (digitalRead(PIN_BUTTON_B) == LOW) {
        if (btnBPressStartMs == 0) btnBPressStartMs = now;
        else if (now - btnBPressStartMs >= 5000) {
            Serial.println("[BUTTON-B] 5s Hold detected: Entering BLE Provisioning Mode!");
            beepBuzzer(500);
            btnBPressStartMs = 0;
            // BLE provisioning trigger
        }
    } else {
        btnBPressStartMs = 0;
    }
}

// ==============================================================================
// SERIAL COMMAND CLI (FOR BENCH TESTING & SIMULATION)
// ==============================================================================

void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "SIM") {
        simulationMode = !simulationMode;
        Serial.printf("[CMD] Simulation Mode: %s\n", simulationMode ? "ENABLED" : "DISABLED");
    } else if (cmd == "CRASH") {
        Serial.println("[CMD] Triggering 5.5G Simulated Collision!");
        collision.update(2.0f, 3.0f, 4.0f, 150.0f, 200.0f, 80.0f, millis());
    } else if (cmd == "CANCEL") {
        Serial.println("[CMD] Rider Cancelled Alarm");
        collision.cancel_alarm();
        digitalWrite(PIN_BUZZER, LOW);
    } else if (cmd == "ALCOHOL") {
        Serial.println("[CMD] Simulated Breathalyzer Alcohol Fail (0.09% BAC)");
        alcohol.update(2900, millis(), true);
    } else if (cmd == "CLEAN") {
        Serial.println("[CMD] Simulated Clean Air / Zero Alcohol");
        alcohol.update(400, millis(), true);
    } else if (cmd == "STATUS") {
        Serial.printf("[STATUS] Worn: %d | BAC: %.3f | Batt: %d%% | Ignition: %s | WS: %d\n",
                      currentFrame.helmet_worn, currentFrame.bac_level, currentFrame.battery_pct,
                      ignition_state_to_str(lastIgnitionState), currentFrame.local_link_connected);
    }
}

// ==============================================================================
// NON-BLOCKING CONNECTION MANAGER
// ==============================================================================

void connectWiFi() {
    Serial.printf("[WIFI] Connecting to %s...\n", wifiSsid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < 5000) {
        delay(250);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        currentFrame.wifi_connected = true;

        if (MDNS.begin("smarthelmet-helmet")) {
            Serial.println("[mDNS] Responder started: smarthelmet-helmet.local");
        }

        // Connect Local WebSocket to Ignition Unit
        IPAddress ignIp;
        if (WiFi.hostByName("smarthelmet-ignition.local", ignIp)) {
            ignitionHost = ignIp.toString();
        }
        wsClient.begin(ignitionHost.c_str(), ignitionPort, "/ws/local");
        wsClient.onEvent(onWebSocketEvent);
        wsClient.setReconnectInterval(2000);
    } else {
        Serial.println("\n[WIFI] Infrastructure AP not found. Trying Ignition SoftAP...");
        WiFi.begin("SH-IGN-0001", "helmet1234");
        // WebSocket client fallback to standard AP gateway
        wsClient.begin("192.168.4.1", 8080, "/ws/local");
        wsClient.onEvent(onWebSocketEvent);
        wsClient.setReconnectInterval(2000);
    }
}

// ==============================================================================
// ARDUINO SETUP & MAIN LOOP
// ==============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==============================================");
    Serial.println(" SMART HELMET EMBEDDED SYSTEM — FIRMWARE v1.0.0 ");
    Serial.println("==============================================");

    // Pin Configurations
    pinMode(PIN_MQ3_AO, INPUT);
    pinMode(PIN_BATTERY_SENSE, INPUT);
    pinMode(PIN_HELMET_IR, INPUT_PULLUP);
    pinMode(PIN_BUTTON_A, INPUT_PULLUP);
    pinMode(PIN_BUTTON_B, INPUT_PULLUP);

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);

    setLeds(false, false, false);
    beepBuzzer(150); // Power-on audible chime

    // Load NVS Preferences
    prefs.begin("helmet_cfg", false);
    deviceId = prefs.getString("dev_id", DEFAULT_DEVICE_ID);
    pairedIgnitionId = prefs.getString("pair_id", DEFAULT_PAIRED_IGNITION_ID);
    pairingPsk = prefs.getString("psk", DEFAULT_PAIRING_PSK);
    wifiSsid = prefs.getString("ssid", "SmartHelmet_AP");
    wifiPass = prefs.getString("pass", "helmet1234");
    prefs.end();

    // Initialize I2C Bus and MPU-6050
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    mpu.initialize();
    if (mpu.testConnection()) {
        mpuPresent = true;
        Serial.println("[MPU-6050] 6-Axis IMU connected and verified OK");
    } else {
        Serial.println("[MPU-6050] Not detected, simulation fallback enabled");
    }

    // Initialize GPS UART2
    gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // Initialize SIM800L GSM Modem
    initSim800L();

    // Start Alcohol Pre-Heat Timer
    alcohol.start_warmup(millis());

    // Connect to Wireless Network
    connectWiFi();

    // Configure Hardware Watchdog (8 seconds timeout)
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);

    Serial.println("[SYSTEM] Initialization complete. Running main supervisor.");
}

void loop() {
    esp_task_wdt_reset(); // Feed hardware watchdog
    wsClient.loop();

    uint32_t now = millis();

    // Poll Hardware Sensors every 50ms
    if (now - lastSensorPollMs >= 50) {
        lastSensorPollMs = now;
        pollSensors();
        handleButtons();
        handleSerialCommands();
    }

    // Transmit Local Authenticated Heartbeat to Ignition every 500ms
    if (now - lastHeartbeatMs >= LOCAL_HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;
        sendLocalHeartbeat();
    }

    // Push Telemetry to Cloud Backend every 3000ms
    if (now - lastCloudTelemetryMs >= CLOUD_TELEMETRY_INTERVAL_MS) {
        lastCloudTelemetryMs = now;
        postCloudTelemetry();
    }

    // Update Status LEDs & UI state
    if (now - lastBlinkMs >= 500) {
        lastBlinkMs = now;
        if (!collision.is_pre_alert() && !collision.is_confirmed_crash()) {
            if (lastIgnitionState == IgnitionState::UNLOCKED || lastIgnitionState == IgnitionState::RUNNING_LATCHED) {
                setLeds(true, false, false); // Solid Green: Safe & Unlocked
            } else if (alcohol.is_warming_up()) {
                setLeds(false, true, false); // Yellow: Warming up MQ-3
            } else if (alcohol.is_over_threshold()) {
                setLeds(false, false, true); // Red: Alcohol Over Limit
            } else if (!currentFrame.helmet_worn) {
                setLeds(false, true, false); // Yellow: Helmet Not Worn
            } else {
                setLeds(false, false, true); // Red: Locked
            }
        }
    }
}