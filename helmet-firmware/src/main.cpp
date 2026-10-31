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
#include "ble_provision.h"

// ==============================================================================
// GLOBAL SYSTEM INSTANCES & STATE
// ==============================================================================

Preferences prefs;
WebSocketsClient wsClient;
MPU6050 mpu(0x68);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
HardwareSerial simSerial(1);

AlcoholProcessor alcohol;
CollisionDetector collision;

HelmetTelemetryFrame currentFrame;
IgnitionState lastIgnitionState = IgnitionState::LOCKED;
LockReason lastLockReason = LockReason::NO_LINK;

String deviceId = DEFAULT_DEVICE_ID;
String pairedIgnitionId = DEFAULT_PAIRED_IGNITION_ID;
String pairingPsk = DEFAULT_PAIRING_PSK;
String wifiSsid = DEFAULT_WIFI_SSID;
String wifiPass = DEFAULT_WIFI_PASS;
String ignitionHost = "192.168.4.1";
int ignitionPort = LOCAL_SERVER_PORT;
String cloudHost = CLOUD_BACKEND_HOST;
int cloudPort = CLOUD_BACKEND_PORT;
bool autoConnect = true;
bool preferSoftAp = true;  // after pairing: helmet↔ignition SoftAP without phone Wi‑Fi

String emergencyContact1 = "+2348000000001";
String emergencyContact2 = "+2348000000002";

uint32_t lastHeartbeatMs = 0;
uint32_t lastCloudTelemetryMs = 0;
uint32_t lastSensorPollMs = 0;
uint32_t lastBlinkMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t bootMs = 0;
uint32_t irWornStartMs = 0;
uint32_t seqCounter = 1;

uint32_t btnAPressStartMs = 0;
uint32_t btnBPressStartMs = 0;
bool btnBProvArmed = false;
bool mpuPresent = false;
bool simPresent = false;
bool simulationMode = false;
bool wsStarted = false;
bool ledPhase = false;

enum class NetLedMode : uint8_t {
    SAFETY,
    BLE_PROV,      // yellow fast blink
    WIFI_TRY,      // yellow slow blink
    SOFTAP_SCAN,   // yellow/red alternate
    LINK_WAIT,     // yellow + green pulse — WiFi up, waiting WS
    LINK_FAIL      // red fast blink
};

NetLedMode netLedMode = NetLedMode::WIFI_TRY;

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

void updateConnectionLeds() {
    uint32_t now = millis();
    uint16_t period = LINK_LED_BLINK_SLOW_MS;

    if (collision.is_pre_alert() || collision.is_confirmed_crash()) {
        return;  // crash path owns LEDs
    }

    switch (netLedMode) {
        case NetLedMode::BLE_PROV:
            period = LINK_LED_BLINK_FAST_MS;
            if (now - lastBlinkMs >= period) {
                lastBlinkMs = now;
                ledPhase = !ledPhase;
                setLeds(false, ledPhase, false);
            }
            break;

        case NetLedMode::WIFI_TRY:
            period = LINK_LED_BLINK_SLOW_MS;
            if (now - lastBlinkMs >= period) {
                lastBlinkMs = now;
                ledPhase = !ledPhase;
                setLeds(false, ledPhase, false);
            }
            break;

        case NetLedMode::SOFTAP_SCAN:
            period = LINK_LED_BLINK_FAST_MS;
            if (now - lastBlinkMs >= period) {
                lastBlinkMs = now;
                ledPhase = !ledPhase;
                setLeds(false, ledPhase, !ledPhase);
            }
            break;

        case NetLedMode::LINK_WAIT:
            period = LINK_LED_BLINK_SLOW_MS;
            if (now - lastBlinkMs >= period) {
                lastBlinkMs = now;
                ledPhase = !ledPhase;
                setLeds(ledPhase, true, false);
            }
            break;

        case NetLedMode::LINK_FAIL:
            period = LINK_LED_BLINK_FAST_MS;
            if (now - lastBlinkMs >= period) {
                lastBlinkMs = now;
                ledPhase = !ledPhase;
                setLeds(false, false, ledPhase);
            }
            break;

        case NetLedMode::SAFETY:
        default:
            // Connection-first LED language (both boards):
            //   GREEN solid     = helmet ↔ ignition linked
            //   GREEN + YELLOW  = peer linked AND cloud/backend OK
            //   YELLOW blink    = warming / not worn
            //   RED             = alcohol fail / no peer / fault
            if (now - lastBlinkMs >= 400) {
                lastBlinkMs = now;
                if (!currentFrame.local_link_connected) {
                    setLeds(false, false, true);
                } else if (alcohol.is_over_threshold()) {
                    setLeds(false, false, true);
                } else if (alcohol.is_warming_up()) {
                    ledPhase = !ledPhase;
                    setLeds(false, ledPhase, false);
                } else if (currentFrame.internet_ok) {
                    setLeds(true, true, false);   // peer + backend
                } else {
                    setLeds(true, false, false);  // peer only (SoftAP OK)
                }
            }
            break;
    }
}

void refreshNetLedMode() {
    if (ble_prov::active) {
        netLedMode = NetLedMode::BLE_PROV;
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
        netLedMode = NetLedMode::WIFI_TRY;
        return;
    }
    if (!currentFrame.local_link_connected) {
        netLedMode = NetLedMode::LINK_WAIT;
        return;
    }
    netLedMode = NetLedMode::SAFETY;
}

// ==============================================================================
// SIM800L GSM/GPRS
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
            Serial.println("[SIM800L] Modem OK");
            simSerial.println("AT+CMGF=1");
            delay(100);
            simSerial.println("AT+CNMI=2,2,0,0,0");
        }
    }
    if (!simPresent) {
        Serial.println("[SIM800L] No response — software fallback");
    }
}

void sendEmergencySms(const String& phone, double lat, double lng, float speed, float impact_g) {
    if (phone.length() == 0) return;
    Serial.printf("[SIM800L] Emergency SMS to %s\n", phone.c_str());

    String msg = "EMERGENCY ALERT: Smart Helmet collision!\n";
    msg += "Impact: " + String(impact_g, 1) + "G\n";
    msg += "Speed: " + String(speed, 1) + " km/h\n";
    msg += "Location: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lng, 6);

    if (simPresent) {
        simSerial.println("AT+CMGS=\"" + phone + "\"");
        delay(300);
        simSerial.print(msg);
        delay(100);
        simSerial.write(26);
        delay(2000);
    } else {
        Serial.println("[SIM800L-SIM] " + msg);
    }
}

// ==============================================================================
// WEBSOCKET LOCAL PROTOCOL
// ==============================================================================

void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            currentFrame.local_link_connected = false;
            lastIgnitionState = IgnitionState::LOCKED;
            lastLockReason = LockReason::NO_LINK;
            Serial.println("[WS-LOCAL] Disconnected from Ignition");
            break;

        case WStype_CONNECTED:
            currentFrame.local_link_connected = true;
            Serial.println("[WS-LOCAL] Connected to Ignition!");
            beepBuzzer(80);
            break;

        case WStype_TEXT: {
            StaticJsonDocument<512> doc;
            if (deserializeJson(doc, payload, length)) return;

            const char* msgType = doc["type"] | "";
            if (strcmp(msgType, "HEARTBEAT_ACK") == 0) {
                const char* stateStr = doc["data"]["state"] | "LOCKED";
                const char* reasonStr = doc["data"]["lock_reason"] | "NONE";

                if (strcmp(stateStr, "UNLOCKED") == 0) lastIgnitionState = IgnitionState::UNLOCKED;
                else if (strcmp(stateStr, "RUNNING_LATCHED") == 0) lastIgnitionState = IgnitionState::RUNNING_LATCHED;
                else if (strcmp(stateStr, "FAULT") == 0) lastIgnitionState = IgnitionState::FAULT;
                else lastIgnitionState = IgnitionState::LOCKED;

                if (strcmp(reasonStr, "NO_HELMET") == 0) lastLockReason = LockReason::NO_HELMET;
                else if (strcmp(reasonStr, "ALCOHOL_DETECTED") == 0) lastLockReason = LockReason::ALCOHOL_DETECTED;
                else if (strcmp(reasonStr, "WARMING_UP") == 0) lastLockReason = LockReason::WARMING_UP;
                else if (strcmp(reasonStr, "CRASH_EVENT") == 0) lastLockReason = LockReason::CRASH_EVENT;
                else if (strcmp(reasonStr, "LOW_BATTERY") == 0) lastLockReason = LockReason::LOW_BATTERY;
                else if (strcmp(reasonStr, "NO_LINK") == 0) lastLockReason = LockReason::NO_LINK;
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
    data["ir_near"] = (currentFrame.worn_duration_ms > 0);
    data["ir_raw"] = digitalRead(PIN_HELMET_IR);
    // Native JSON numbers so ignition SoftAP path never drops IMU/GPS to 0
    data["bac"] = currentFrame.bac_level;
    data["mq3_raw"] = currentFrame.mq3_raw;
    data["mq3_warm"] = currentFrame.mq3_warmed_up;
    data["impact"] = currentFrame.impact_flag;
    data["impact_g"] = currentFrame.impact_g;
    data["tilt_deg"] = currentFrame.tilt_deg;
    data["tilt_warn"] = currentFrame.tilt_warning;
    data["imu_anomaly"] = currentFrame.imu_anomaly;
    data["imu_state"] = currentFrame.imu_state;
    data["ax"] = currentFrame.ax;
    data["ay"] = currentFrame.ay;
    data["az"] = currentFrame.az;
    data["gx"] = currentFrame.gx;
    data["gy"] = currentFrame.gy;
    data["gz"] = currentFrame.gz;
    data["mpu_ok"] = mpuPresent;
    data["alcohol_rise"] = currentFrame.alcohol_rising_anomaly;
    data["rider_upright"] = currentFrame.rider_upright;
    data["batt_pct"] = currentFrame.battery_pct;
    data["batt_mv"] = currentFrame.battery_mv;
    data["gps_fix"] = currentFrame.gps_fixed;
    data["lat"] = currentFrame.latitude;
    data["lon"] = currentFrame.longitude;
    data["speed"] = currentFrame.speed_kmh;
    data["wifi_mode"] = currentFrame.wifi_mode;

    String dataString;
    serializeJson(data, dataString);

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

void startWebSocketClient() {
    wsClient.begin(ignitionHost.c_str(), ignitionPort, "/ws/local");
    wsClient.onEvent(onWebSocketEvent);
    wsClient.setReconnectInterval(2000);
    wsStarted = true;
    Serial.printf("[WS-LOCAL] Connecting to %s:%d/ws/local\n", ignitionHost.c_str(), ignitionPort);
}

// ==============================================================================
// CLOUD TELEMETRY
// ==============================================================================

void postCloudTelemetry() {
    if (WiFi.status() != WL_CONNECTED) {
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
#if CLOUD_USE_HTTPS
    String url = "https://" + cloudHost + CLOUD_TELEMETRY_ENDPOINT;
#else
    String url = "http://" + cloudHost + ":" + String(cloudPort) + CLOUD_TELEMETRY_ENDPOINT;
#endif
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    while (queueCount > 0) {
        String queuedPayload = offlineQueue[queueHead];
        queueHead = (queueHead + 1) % OFFLINE_QUEUE_CAPACITY;
        queueCount--;
        http.POST(queuedPayload);
    }

    StaticJsonDocument<1024> doc;
    doc["device_id"] = deviceId;
    doc["helmet_worn"] = currentFrame.helmet_worn;
    doc["bac_level"] = currentFrame.bac_level;
    doc["alcohol_detected"] = alcohol.is_over_threshold();
    doc["alcohol_warmup_pct"] = currentFrame.alcohol_warmup_pct;
    doc["alcohol_calibrated"] = currentFrame.alcohol_calibrated;
    doc["bac_rise_rate"] = currentFrame.bac_rise_rate;
    doc["alcohol_rising_anomaly"] = currentFrame.alcohol_rising_anomaly;
    doc["mq3_raw"] = currentFrame.mq3_raw;
    doc["latitude"] = currentFrame.latitude;
    doc["longitude"] = currentFrame.longitude;
    doc["speed_kmh"] = currentFrame.speed_kmh;
    doc["battery_percentage"] = currentFrame.battery_pct;
    doc["battery_voltage"] = currentFrame.battery_mv / 1000.0f;
    doc["impact_g"] = currentFrame.impact_g;
    doc["tilt_degrees"] = currentFrame.tilt_deg;
    doc["roll_degrees"] = currentFrame.roll_deg;
    doc["pitch_degrees"] = currentFrame.pitch_deg;
    doc["ax"] = currentFrame.ax;
    doc["ay"] = currentFrame.ay;
    doc["az"] = currentFrame.az;
    doc["gx"] = currentFrame.gx;
    doc["gy"] = currentFrame.gy;
    doc["gz"] = currentFrame.gz;
    doc["imu_state"] = currentFrame.imu_state;
    doc["imu_anomaly"] = currentFrame.imu_anomaly;
    doc["tilt_warning"] = currentFrame.tilt_warning;
    doc["rider_upright"] = currentFrame.rider_upright;
    doc["ignition_status"] = ignition_state_to_str(lastIgnitionState);
    doc["lock_reason"] = lock_reason_to_str(lastLockReason);
    doc["local_link_connected"] = currentFrame.local_link_connected;
    doc["wifi_mode"] = currentFrame.wifi_mode;
    doc["wifi_ssid"] = WiFi.SSID();
    doc["internet_ok"] = currentFrame.internet_ok;
    doc["shared_wifi"] = (strcmp(currentFrame.wifi_mode, "STA") == 0);
    doc["prefer_softap"] = preferSoftAp;
    doc["rssi"] = WiFi.RSSI();
    doc["link_state"] = currentFrame.local_link_connected ? "CONNECTED" : "NO_LINK";

    String payload;
    serializeJson(doc, payload);

    int code = http.POST(payload);
    currentFrame.internet_ok = (code > 0 && code < 500);
    if (code <= 0) {
        enqueueOfflineTelemetry(payload);
        currentFrame.internet_ok = false;
    }
    http.end();
}

void postCloudEmergencyEvent(const char* eventType, const char* desc) {
    if (WiFi.status() != WL_CONNECTED) return;

    HTTPClient http;
#if CLOUD_USE_HTTPS
    String url = "https://" + cloudHost + CLOUD_EVENTS_ENDPOINT;
#else
    String url = "http://" + cloudHost + ":" + String(cloudPort) + CLOUD_EVENTS_ENDPOINT;
#endif
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
// SENSOR ACQUISITION
// ==============================================================================

void pollSensors() {
    uint32_t now = millis();

    // FC-51 digital out: factory boards are usually Active-LOW when object near.
    // Also accept Active-HIGH modules via NVS flag ir_active_high.
    static int8_t irActiveHigh = -1;
    if (irActiveHigh < 0) {
        prefs.begin("helmet_cfg", true);
        irActiveHigh = prefs.getBool("ir_active_high", false) ? 1 : 0;
        prefs.end();
    }
    int irLevel = digitalRead(PIN_HELMET_IR);
    bool rawNear = irActiveHigh ? (irLevel == HIGH) : (irLevel == LOW);
    if (simulationMode) rawNear = true;

    if (rawNear) {
        if (irWornStartMs == 0) irWornStartMs = now;
        currentFrame.worn_duration_ms = now - irWornStartMs;
        // Instant near-flag for app diagnostics; unlock still needs 1s stable
        currentFrame.helmet_worn = (currentFrame.worn_duration_ms >= 1000);
    } else {
        irWornStartMs = 0;
        currentFrame.worn_duration_ms = 0;
        currentFrame.helmet_worn = false;
    }

    uint16_t rawAdc = analogRead(PIN_MQ3_AO);
    if (simulationMode && rawAdc < 300) rawAdc = 420;
    currentFrame.mq3_raw = rawAdc;

    alcohol.update(rawAdc, now, currentFrame.helmet_worn);
    currentFrame.bac_level = alcohol.get_bac();
    currentFrame.mq3_warmed_up = !alcohol.is_warming_up();

    uint16_t rawBatt = analogRead(PIN_BATTERY_SENSE);
    float vBattAdc = (rawBatt / 4095.0f) * 3.3f;
    float vBattActual = vBattAdc * 2.0f;
    if (vBattActual < 2.5f) vBattActual = 4.05f;
    currentFrame.battery_mv = (uint16_t)(vBattActual * 1000.0f);
    float pct = ((vBattActual - 3.4f) / (4.2f - 3.4f)) * 100.0f;
    currentFrame.battery_pct = (uint8_t)constrain((int)pct, 0, 100);

    float ax = 0.0f, ay = 0.0f, az = 1.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    if (!mpuPresent) {
        static uint32_t lastMpuRetryMs = 0;
        if (now - lastMpuRetryMs > 5000) {
            lastMpuRetryMs = now;
            mpu.initialize();
            mpuPresent = mpu.testConnection();
            if (mpuPresent) Serial.println("[MPU-6050] Recovered");
        }
    }
    if (mpuPresent) {
        int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
        mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);
        ax = rawAx / 16384.0f;
        ay = rawAy / 16384.0f;
        az = rawAz / 16384.0f;
        gx = rawGx / 131.0f;
        gy = rawGy / 131.0f;
        gz = rawGz / 131.0f;
    } else if (simulationMode) {
        // Serial "SIM" — synthetic motion so UI pipeline can be verified without MPU
        ax = 0.15f * sinf(now / 400.0f);
        ay = 0.12f * cosf(now / 350.0f);
        az = 0.95f + 0.05f * sinf(now / 500.0f);
        gx = 40.0f * sinf(now / 300.0f);
        gy = 25.0f * cosf(now / 280.0f);
        gz = 10.0f * sinf(now / 450.0f);
    }

    collision.update(ax, ay, az, gx, gy, gz, now);
    {
        static uint32_t lastImuLogMs = 0;
        if ((mpuPresent || simulationMode) && now - lastImuLogMs > 2000) {
            lastImuLogMs = now;
            Serial.printf("[IMU] ax=%.2f ay=%.2f az=%.2f tilt=%.1f° mpu=%d sim=%d\n",
                          ax, ay, az, collision.get_tilt_deg(),
                          mpuPresent ? 1 : 0, simulationMode ? 1 : 0);
        }
    }
    currentFrame.impact_g = collision.get_peak_impact_g();
    currentFrame.tilt_deg = collision.get_tilt_deg();
    currentFrame.roll_deg = collision.get_roll_deg();
    currentFrame.pitch_deg = collision.get_pitch_deg();
    currentFrame.ax = collision.get_ax();
    currentFrame.ay = collision.get_ay();
    currentFrame.az = collision.get_az();
    currentFrame.gx = collision.get_gx();
    currentFrame.gy = collision.get_gy();
    currentFrame.gz = collision.get_gz();
    currentFrame.imu_anomaly = collision.is_anomaly();
    currentFrame.tilt_warning = collision.is_tilt_warning();
    currentFrame.rider_upright = collision.is_rider_upright();
    currentFrame.imu_state = collision.state_str();
    currentFrame.impact_flag = collision.is_confirmed_crash();
    currentFrame.alcohol_warmup_pct = alcohol.warmup_progress_pct(now);
    currentFrame.bac_rise_rate = alcohol.get_bac_rise_rate();
    currentFrame.alcohol_rising_anomaly = alcohol.is_rising_anomaly();
    currentFrame.alcohol_calibrated = alcohol.is_calibrated();

    if (collision.is_pre_alert()) {
        if ((now / 250) % 2 == 0) digitalWrite(PIN_BUZZER, HIGH);
        else digitalWrite(PIN_BUZZER, LOW);
        setLeds(false, false, true);
    } else if (collision.is_confirmed_crash()) {
        digitalWrite(PIN_BUZZER, HIGH);
        setLeds(false, false, true);
        static bool emergencySent = false;
        if (!emergencySent) {
            emergencySent = true;
            sendEmergencySms(emergencyContact1, currentFrame.latitude, currentFrame.longitude,
                             currentFrame.speed_kmh, currentFrame.impact_g);
            sendEmergencySms(emergencyContact2, currentFrame.latitude, currentFrame.longitude,
                             currentFrame.speed_kmh, currentFrame.impact_g);
            postCloudEmergencyEvent("COLLISION", "Severe collision detected with rider fall");
        }
    }

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
// BLE + BUTTONS
// ==============================================================================

void enterBleProvisioning() {
    if (ble_prov::active) return;
    Serial.println("[BLE] Entering app provisioning (SH-HLM-PROV)");
    if (wsStarted) {
        wsClient.disconnect();
        wsStarted = false;
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    currentFrame.wifi_connected = false;
    currentFrame.local_link_connected = false;
    ble_prov::start(BLE_ADV_NAME);
    netLedMode = NetLedMode::BLE_PROV;
    beepBuzzer(200);
}

void handleButtons() {
    uint32_t now = millis();

    if (digitalRead(PIN_BUTTON_A) == LOW) {
        if (btnAPressStartMs == 0) btnAPressStartMs = now;
    } else {
        if (btnAPressStartMs > 0) {
            uint32_t duration = now - btnAPressStartMs;
            if (duration >= 50 && duration < 3000) {
                if (collision.is_pre_alert()) {
                    collision.cancel_alarm();
                    digitalWrite(PIN_BUZZER, LOW);
                    Serial.println("[BUTTON-A] False alarm cancelled");
                    beepBuzzer(100);
                }
            }
            btnAPressStartMs = 0;
        }
    }

    if (digitalRead(PIN_BUTTON_B) == LOW) {
        if (btnBPressStartMs == 0) {
            btnBPressStartMs = now;
            btnBProvArmed = false;
        } else if (!btnBProvArmed && now - btnBPressStartMs >= 3000) {
            btnBProvArmed = true;
            enterBleProvisioning();
        }
    } else {
        btnBPressStartMs = 0;
        btnBProvArmed = false;
    }
}

void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "SIM") {
        simulationMode = !simulationMode;
        Serial.printf("[CMD] Simulation: %s\n", simulationMode ? "ON" : "OFF");
    } else if (cmd == "CRASH") {
        collision.update(2.0f, 3.0f, 4.0f, 150.0f, 200.0f, 80.0f, millis());
    } else if (cmd == "CANCEL") {
        collision.cancel_alarm();
        digitalWrite(PIN_BUZZER, LOW);
    } else if (cmd == "ALCOHOL") {
        alcohol.update(2900, millis(), true);
    } else if (cmd == "CLEAN") {
        alcohol.update(400, millis(), true);
    } else if (cmd == "PROV" || cmd == "BLE") {
        enterBleProvisioning();
    } else if (cmd == "IRINV") {
        prefs.begin("helmet_cfg", false);
        bool cur = prefs.getBool("ir_active_high", false);
        prefs.putBool("ir_active_high", !cur);
        prefs.end();
        Serial.printf("[IR] Active-HIGH polarity now %s — reboot or wait next poll\n", !cur ? "ON" : "OFF");
    } else if (cmd == "STATUS") {
        Serial.printf("[STATUS] Worn:%d BAC:%.3f Batt:%d%% Ign:%s WiFi:%d WS:%d BLE:%d\n",
                      currentFrame.helmet_worn, currentFrame.bac_level, currentFrame.battery_pct,
                      ignition_state_to_str(lastIgnitionState),
                      currentFrame.wifi_connected, currentFrame.local_link_connected, ble_prov::active);
    }
}

// ==============================================================================
// WIFI: STA → scan SoftAP SH-IGN-* → reconnect / auto BLE
// ==============================================================================

bool tryConnectSsid(const char* ssid, const char* pass, uint32_t timeoutMs) {
    Serial.printf("[WIFI] Connecting to %s...\n", ssid);
    netLedMode = NetLedMode::WIFI_TRY;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(200);
        updateConnectionLeds();
        esp_task_wdt_reset();
        Serial.print(".");
    }
    Serial.println();
    return WiFi.status() == WL_CONNECTED;
}

String scanIgnitionSoftAp() {
    netLedMode = NetLedMode::SOFTAP_SCAN;
    Serial.println("[WIFI] Scanning for ignition SoftAP (SH-IGN-*)...");
    int n = WiFi.scanNetworks(false, false);
    String found;
    for (int i = 0; i < n; i++) {
        String s = WiFi.SSID(i);
        if (s.startsWith(DEFAULT_SOFTAP_PREFIX)) {
            found = s;
            Serial.printf("[WIFI] Found SoftAP: %s (RSSI %d)\n", s.c_str(), WiFi.RSSI(i));
            break;
        }
    }
    WiFi.scanDelete();
    return found;
}

bool softApActiveGateway() {
    String ssid = WiFi.SSID();
    return ssid.startsWith(DEFAULT_SOFTAP_PREFIX);
}

void resolveIgnitionHost() {
    IPAddress ignIp;
    if (WiFi.hostByName("smarthelmet-ignition.local", ignIp)) {
        ignitionHost = ignIp.toString();
        Serial.printf("[mDNS] Ignition at %s\n", ignitionHost.c_str());
    } else if (softApActiveGateway()) {
        ignitionHost = "192.168.4.1";
    } else {
        ignitionHost = WiFi.gatewayIP().toString();
        if (ignitionHost == "0.0.0.0") ignitionHost = "192.168.4.1";
    }
}

bool isSoftApOnlySsid(const String& ssid) {
    return ssid.length() == 0 || ssid == SOFTAP_ONLY_SSID || preferSoftAp;
}

bool connectSoftApPeer() {
    // Prefer last SoftAP name, then scan SH-IGN-*
    prefs.begin("helmet_cfg", true);
    String lastSoft = prefs.getString("last_softap", "");
    prefs.end();

    if (lastSoft.length() > 0) {
        if (tryConnectSsid(lastSoft.c_str(), DEFAULT_SOFTAP_PASS, 8000)) {
            currentFrame.wifi_connected = true;
            currentFrame.wifi_mode = "SOFTAP";
            ignitionHost = "192.168.4.1";
            startWebSocketClient();
            Serial.printf("[WIFI] SoftAP peer (cached): %s\n", lastSoft.c_str());
            return true;
        }
    }

    String softAp = scanIgnitionSoftAp();
    if (softAp.length() > 0 &&
        tryConnectSsid(softAp.c_str(), DEFAULT_SOFTAP_PASS, WIFI_STA_TIMEOUT_MS)) {
        currentFrame.wifi_connected = true;
        currentFrame.wifi_mode = "SOFTAP";
        ignitionHost = "192.168.4.1";
        prefs.begin("helmet_cfg", false);
        prefs.putString("last_softap", softAp);
        prefs.end();
        startWebSocketClient();
        Serial.printf("[WIFI] SoftAP peer linked: %s (no phone Wi‑Fi needed)\n", softAp.c_str());
        return true;
    }
    return false;
}

void connectWiFi() {
    if (!autoConnect) {
        Serial.println("[WIFI] Auto-connect off — opening BLE for app setup");
        enterBleProvisioning();
        return;
    }

    lastWifiAttemptMs = millis();
    currentFrame.prefer_softap = preferSoftAp;

    // DEFAULT after pairing: SoftAP peer first so ignition works without phone/internet
    if (isSoftApOnlySsid(wifiSsid)) {
        Serial.println("[WIFI] SoftAP-prefer peer mode (helmet ↔ ignition direct)");
        if (connectSoftApPeer()) return;
        Serial.println("[WIFI] SoftAP peer not found — will retry");
        currentFrame.wifi_connected = false;
        currentFrame.wifi_mode = "NONE";
        netLedMode = NetLedMode::LINK_FAIL;
        return;
    }

    // Optional shared infrastructure Wi‑Fi (app monitoring / cloud)
    if (tryConnectSsid(wifiSsid.c_str(), wifiPass.c_str(), WIFI_STA_TIMEOUT_MS)) {
        Serial.printf("[WIFI] STA Connected! IP: %s SSID: %s\n",
                      WiFi.localIP().toString().c_str(), WiFi.SSID().c_str());
        currentFrame.wifi_connected = true;
        currentFrame.wifi_mode = "STA";
        MDNS.begin("smarthelmet-helmet");
        resolveIgnitionHost();
        startWebSocketClient();
        return;
    }

    // Fallback to SoftAP peer if shared Wi‑Fi missing
    if (connectSoftApPeer()) return;

    Serial.println("[WIFI] No network — will retry / open BLE if link stays down");
    currentFrame.wifi_connected = false;
    currentFrame.wifi_mode = "NONE";
    netLedMode = NetLedMode::LINK_FAIL;
}

void maintainWifiAndBle() {
    if (ble_prov::active) {
        if (ble_prov::applyToNvs(prefs, "helmet_cfg", wifiSsid, wifiPass, pairingPsk, pairedIgnitionId)) {
            ESP.restart();
        }
        return;
    }

    uint32_t now = millis();

    if (WiFi.status() != WL_CONNECTED) {
        currentFrame.wifi_connected = false;
        currentFrame.local_link_connected = false;
        if (now - lastWifiAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
            connectWiFi();
        }
    } else {
        currentFrame.wifi_connected = true;
        if (!wsStarted) {
            resolveIgnitionHost();
            startWebSocketClient();
        }
    }

    // Auto BLE if never got a local link (no LCD needed for pairing)
    if (!currentFrame.local_link_connected &&
        (now - bootMs) >= AUTO_BLE_AFTER_NO_LINK_MS &&
        !ble_prov::active) {
        static bool autoBleDone = false;
        if (!autoBleDone) {
            autoBleDone = true;
            Serial.println("[BLE] No ignition link — auto-opening BLE for app pairing");
            enterBleProvisioning();
        }
    }
}

// ==============================================================================
// SETUP & LOOP
// ==============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==============================================");
    Serial.println(" SMART HELMET — FIRMWARE v1.1.0 ");
    Serial.println(" App BLE pairing | LED link status | Auto SoftAP");
    Serial.println("==============================================");

    bootMs = millis();

    pinMode(PIN_MQ3_AO, INPUT);
    pinMode(PIN_BATTERY_SENSE, INPUT);
    pinMode(PIN_HELMET_IR, INPUT_PULLUP);
    pinMode(PIN_BUTTON_A, INPUT_PULLUP);
    pinMode(PIN_BUTTON_B, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);

    setLeds(false, true, false);  // yellow = booting / connecting
    beepBuzzer(150);

    prefs.begin("helmet_cfg", false);
    deviceId = prefs.getString("dev_id", DEFAULT_DEVICE_ID);
    pairedIgnitionId = prefs.getString("pair_id", DEFAULT_PAIRED_IGNITION_ID);
    pairingPsk = prefs.getString("psk", DEFAULT_PAIRING_PSK);
    wifiSsid = prefs.getString("ssid", SOFTAP_ONLY_SSID);
    wifiPass = prefs.getString("pass", DEFAULT_SOFTAP_PASS);
    autoConnect = prefs.getBool("auto_connect", true);
    preferSoftAp = prefs.getBool("prefer_softap", true);
    bool wasProvisioned = prefs.getBool("provisioned", false);
    if (wifiSsid == SOFTAP_ONLY_SSID || wifiSsid.length() == 0) preferSoftAp = true;
    prefs.end();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);
    delay(80);
    Serial.printf("[I2C] Scanning SDA=%d SCL=%d ...\n", PIN_I2C_SDA, PIN_I2C_SCL);
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C] device @ 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) Serial.println("[I2C] no devices — check MPU wiring / 3V3 / pull-ups");

    auto tryMpu = [](uint8_t addr) -> bool {
        MPU6050 probe(addr);
        probe.initialize();
        delay(40);
        if (!probe.testConnection()) return false;
        // Wake + ranges (some clones boot asleep)
        probe.setSleepEnabled(false);
        probe.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        probe.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        mpu = probe;
        Serial.printf("[MPU-6050] OK @ 0x%02X (100kHz)\n", addr);
        return true;
    };

    mpuPresent = tryMpu(0x68) || tryMpu(0x69);
    if (!mpuPresent) {
        delay(200);
        mpuPresent = tryMpu(0x68) || tryMpu(0x69);
        if (!mpuPresent) {
            Serial.println("[MPU-6050] Missing — type SIM on serial to fake motion, or rewire AD0/SDA/SCL");
        }
    }

    gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    initSim800L();
    alcohol.start_warmup(millis());

    // Fresh boards: advertise BLE immediately so the phone can pair without waiting
    if (!wasProvisioned || !autoConnect) {
        Serial.println("[BLE] Not provisioned / auto-connect off — advertising SH-HLM-PROV");
        enterBleProvisioning();
    } else {
        connectWiFi();
    }

    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);

    Serial.printf("[SYSTEM] Device=%s PairedIgn=%s AutoConnect=%d\n",
                  deviceId.c_str(), pairedIgnitionId.c_str(), autoConnect);
}

void loop() {
    esp_task_wdt_reset();
    if (wsStarted && !ble_prov::active) {
        wsClient.loop();
    }

    uint32_t now = millis();

    if (now - lastSensorPollMs >= 50) {
        lastSensorPollMs = now;
        if (!ble_prov::active) {
            pollSensors();
        }
        handleButtons();
        handleSerialCommands();
        maintainWifiAndBle();
        refreshNetLedMode();
        updateConnectionLeds();
    }

    if (!ble_prov::active && now - lastHeartbeatMs >= LOCAL_HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;
        sendLocalHeartbeat();
    }

    if (!ble_prov::active && now - lastCloudTelemetryMs >= CLOUD_TELEMETRY_INTERVAL_MS) {
        lastCloudTelemetryMs = now;
        postCloudTelemetry();
    }
}
