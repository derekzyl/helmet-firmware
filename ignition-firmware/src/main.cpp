#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include "pins.h"
#include "config.h"
#include "crypto_utils.h"
#include "safety_fsm.h"

// ==============================================================================
// GLOBAL SYSTEM INSTANCES & STATE
// ==============================================================================

Preferences prefs;
WebSocketsServer wsServer(LOCAL_SERVER_PORT);
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);

SafetyFSM safety;
HeartbeatInput latestHeartbeat;

// Configuration & Credentials (loaded from NVS)
String deviceId = DEFAULT_DEVICE_ID;
String pairedHelmetId = DEFAULT_PAIRED_HELMET_ID;
String pairingPsk = DEFAULT_PAIRING_PSK;
String wifiSsid = "SmartHelmet_AP";
String wifiPass = "helmet1234";

// Operational Variables
float v12BatteryVolts = 12.6f;
uint8_t lcdCurrentPage = 0;
bool lcdDetected = false;
bool softApActive = false;
uint32_t lastSeqReceived = 0;
uint32_t lastLcdUpdateMs = 0;
uint32_t lastAnalogSenseMs = 0;
uint32_t lastLedBlinkMs = 0;
uint32_t btn1PressStartMs = 0;
uint32_t btn2PressStartMs = 0;

// ==============================================================================
// HARDWARE ACTUATION & I/O
// ==============================================================================

void updateActuators() {
    bool ssrActive = safety.is_ssr_active();
    // Safety Rule: GPIO 25 is HIGH only when safe
    digitalWrite(PIN_SSR_CONTROL, ssrActive ? HIGH : LOW);

    // Status Indicator LEDs
    if (safety.get_state() == IgnitionState::UNLOCKED || safety.get_state() == IgnitionState::RUNNING_LATCHED) {
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
    } else {
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, HIGH);
    }

    // Buzzer warning during mid-ride alert or fault
    if (safety.is_mid_ride_warning()) {
        static uint32_t buzzTimer = 0;
        if (millis() - buzzTimer > 500) {
            buzzTimer = millis();
            digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
        }
    } else if (safety.get_state() == IgnitionState::FAULT) {
        digitalWrite(PIN_BUZZER, HIGH);
    } else {
        digitalWrite(PIN_BUZZER, LOW);
    }
}

// ==============================================================================
// 12V MOTORCYCLE BATTERY SENSE
// ==============================================================================

void read12VBattery() {
    uint16_t rawAdc = analogRead(PIN_12V_SENSE);
    float vAdc = (rawAdc / 4095.0f) * 3.3f;
    float vActual = vAdc * ADC_12V_DIVIDER_RATIO;
    // Bench fallback if disconnected
    if (vActual < 2.0f) vActual = 12.6f;
    v12BatteryVolts = vActual;
}

// ==============================================================================
// 16x2 LCD DISPLAY PAGES (CYCLING VIA BUTTON 1)
// ==============================================================================

void initLcd() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
    // Probe I2C address 0x27, then 0x3F
    Wire.beginTransmission(0x27);
    if (Wire.endTransmission() == 0) {
        lcd = LiquidCrystal_I2C(0x27, LCD_COLUMNS, LCD_ROWS);
        lcdDetected = true;
    } else {
        Wire.beginTransmission(0x3F);
        if (Wire.endTransmission() == 0) {
            lcd = LiquidCrystal_I2C(0x3F, LCD_COLUMNS, LCD_ROWS);
            lcdDetected = true;
        }
    }

    if (lcdDetected) {
        lcd.init();
        lcd.backlight();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SMART HELMET SYS");
        lcd.setCursor(0, 1);
        lcd.print("IGNITION v1.0.0");
    } else {
        Serial.println("[LCD] I2C Backpack not detected on 0x27 or 0x3F");
    }
}

void renderLcdPage() {
    if (!lcdDetected) return;

    lcd.clear();
    switch (lcdCurrentPage) {
        case 0: // Status & Lock Reason / SSR
            lcd.setCursor(0, 0);
            lcd.printf("ST:%s", ignition_state_to_str(safety.get_state()));
            lcd.setCursor(0, 1);
            if (safety.is_ssr_active()) {
                lcd.print("SSR:ENGAGED (ON)");
            } else {
                lcd.printf("RSN:%s", lock_reason_to_str(safety.get_lock_reason()));
            }
            break;

        case 1: // Link Status & Network IP
            lcd.setCursor(0, 0);
            if (softApActive) {
                lcd.print("NET:SoftAP MODE");
            } else if (WiFi.status() == WL_CONNECTED) {
                lcd.printf("SSID:%s", WiFi.SSID().substring(0, 10).c_str());
            } else {
                lcd.print("NET:CONNECTING");
            }
            lcd.setCursor(0, 1);
            lcd.printf("IP:%s", (softApActive ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str());
            break;

        case 2: // Helmet Battery & Alcohol BAC
            lcd.setCursor(0, 0);
            lcd.printf("HLM BATT: %d%%", latestHeartbeat.batt_pct);
            lcd.setCursor(0, 1);
            lcd.printf("BAC: %.3f%% %s", latestHeartbeat.bac,
                       (latestHeartbeat.bac > 0.05f) ? "FAIL" : "PASS");
            break;

        case 3: // 12V Motorcycle Battery
            lcd.setCursor(0, 0);
            lcd.print("BIKE 12V BATTERY");
            lcd.setCursor(0, 1);
            lcd.printf("VOLTS: %.2fV %s", v12BatteryVolts,
                       (v12BatteryVolts < V12_CRITICAL_LOW_VOLTS) ? "LOW" : "OK");
            break;

        case 4: // Uptime & Frame Counter
            lcd.setCursor(0, 0);
            lcd.printf("UP: %lus", millis() / 1000);
            lcd.setCursor(0, 1);
            lcd.printf("NONCE: #%lu", lastSeqReceived);
            break;
    }
}

// ==============================================================================
// LOCAL WEBSOCKET PROTOCOL SERVER
// ==============================================================================

void onWebSocketServerEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WS-SRV] Client #%u Disconnected\n", num);
            break;

        case WStype_CONNECTED:
            Serial.printf("[WS-SRV] Client #%u Connected from IP\n", num);
            break;

        case WStype_TEXT: {
            StaticJsonDocument<768> doc;
            DeserializationError err = deserializeJson(doc, payload, length);
            if (err) {
                Serial.println("[WS-SRV] Invalid JSON frame received");
                return;
            }

            const char* msgType = doc["type"] | "";
            const char* senderDev = doc["dev"] | "";
            uint32_t seq = doc["seq"] | 0;
            uint32_t ts = doc["ts"] | 0;
            const char* sig = doc["sig"] | "";

            // 1. Authenticate Paired Device ID
            if (String(senderDev) != pairedHelmetId) {
                Serial.printf("[WS-SRV] REJECTED: Unpaired device %s (expected %s)\n",
                              senderDev, pairedHelmetId.c_str());
                return;
            }

            // 2. Anti-Replay: Verify Monotonic Sequence Nonce
            if (seq <= lastSeqReceived && lastSeqReceived > 0) {
                Serial.printf("[WS-SRV] REJECTED: Replay attack detected! seq=%lu <= last=%lu\n",
                              seq, lastSeqReceived);
                return;
            }

            // 3. Verify HMAC-SHA256 Signature
            JsonObject data = doc["data"];
            String dataString;
            serializeJson(data, dataString);

            String sigPayload = String(doc["ver"].as<int>()) + ":" +
                                String(msgType) + ":" +
                                String(senderDev) + ":" +
                                String(seq) + ":" +
                                String(ts) + ":" +
                                dataString;

            if (!crypto::verify_hmac(pairingPsk.c_str(), sigPayload.c_str(), sig)) {
                Serial.println("[WS-SRV] REJECTED: Invalid HMAC-SHA256 signature!");
                return;
            }

            // Signature & Nonce Validated!
            lastSeqReceived = seq;
            safety.register_heartbeat(millis());

            if (strcmp(msgType, "HEARTBEAT") == 0) {
                latestHeartbeat.worn = data["worn"] | false;
                latestHeartbeat.worn_ms = data["worn_ms"] | 0;
                latestHeartbeat.bac = data["bac"] | 0.0f;
                latestHeartbeat.mq3_warm = data["mq3_warm"] | false;
                latestHeartbeat.impact = data["impact"] | false;
                latestHeartbeat.batt_pct = data["batt_pct"] | 100;
                latestHeartbeat.speed_kmh = data["speed"] | 0.0f;

                // Send HEARTBEAT_ACK
                StaticJsonDocument<384> ackDoc;
                ackDoc["ver"] = 1;
                ackDoc["type"] = "HEARTBEAT_ACK";
                ackDoc["dev"] = deviceId;
                ackDoc["seq"] = seq;
                ackDoc["ts"] = (uint32_t)(millis() / 1000);

                JsonObject ackData = ackDoc.createNestedObject("data");
                ackData["state"] = ignition_state_to_str(safety.get_state());
                ackData["ssr_active"] = safety.is_ssr_active();
                ackData["lock_reason"] = lock_reason_to_str(safety.get_lock_reason());
                ackData["v12_mv"] = (uint16_t)(v12BatteryVolts * 1000.0f);
                ackData["err_code"] = 0;

                String ackDataString;
                serializeJson(ackData, ackDataString);

                String ackSigPayload = String(ackDoc["ver"].as<int>()) + ":" +
                                       "HEARTBEAT_ACK:" +
                                       deviceId + ":" +
                                       String(seq) + ":" +
                                       String(ackDoc["ts"].as<uint32_t>()) + ":" +
                                       ackDataString;

                ackDoc["sig"] = crypto::hmac_sha256_hex(pairingPsk.c_str(), ackSigPayload.c_str()).c_str();

                String ackOutput;
                serializeJson(ackDoc, ackOutput);
                wsServer.sendTXT(num, ackOutput);
            } else if (strcmp(msgType, "MAINTENANCE_OVERRIDE") == 0) {
                uint32_t duration = data["duration_sec"] | 300;
                safety.apply_maintenance_override(millis(), duration);
                Serial.printf("[WS-SRV] Maintenance Override Activated for %lu seconds\n", duration);
            }
            break;
        }

        default:
            break;
    }
}

// ==============================================================================
// BUTTON INPUT HANDLING
// ==============================================================================

void handleButtons() {
    uint32_t now = millis();

    // Button 1: Cycle LCD Display Pages
    if (digitalRead(PIN_BUTTON_1) == LOW) {
        if (btn1PressStartMs == 0) btn1PressStartMs = now;
    } else {
        if (btn1PressStartMs > 0) {
            uint32_t duration = now - btn1PressStartMs;
            if (duration >= 50 && duration < 2000) {
                lcdCurrentPage = (lcdCurrentPage + 1) % TOTAL_LCD_PAGES;
                renderLcdPage();
                Serial.printf("[BUTTON-1] Cycled to LCD Page %d\n", lcdCurrentPage);
            }
            btn1PressStartMs = 0;
        }
    }

    // Button 2: Hold 5s for Provisioning / Hold 15s for Factory Reset
    if (digitalRead(PIN_BUTTON_2) == LOW) {
        if (btn2PressStartMs == 0) btn2PressStartMs = now;
        else if (now - btn2PressStartMs >= 15000) {
            Serial.println("[BUTTON-2] 15s Hold: FACTORY RESET!");
            prefs.begin("ignition_cfg", false);
            prefs.clear();
            prefs.end();
            btn2PressStartMs = 0;
            ESP.restart();
        } else if (now - btn2PressStartMs >= 5000 && now - btn2PressStartMs < 5200) {
            Serial.println("[BUTTON-2] 5s Hold: BLE Provisioning Mode Triggered");
        }
    } else {
        btn2PressStartMs = 0;
    }
}

// ==============================================================================
// SERIAL COMMAND CLI (FOR BENCH TESTING & SIMULATION)
// ==============================================================================

void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "UNLOCK") {
        Serial.println("[CMD] Manually Simulating Safe Rider Unlock");
        latestHeartbeat.worn = true;
        latestHeartbeat.worn_ms = 2000;
        latestHeartbeat.mq3_warm = true;
        latestHeartbeat.bac = 0.00f;
        latestHeartbeat.batt_pct = 95;
        safety.register_heartbeat(millis());
    } else if (cmd == "ALCOHOL") {
        Serial.println("[CMD] Injecting Alcohol Violation (0.09% BAC)");
        latestHeartbeat.bac = 0.09f;
        safety.register_heartbeat(millis());
    } else if (cmd == "CRASH") {
        Serial.println("[CMD] Injecting Collision Cutoff Event");
        latestHeartbeat.impact = true;
        safety.register_heartbeat(millis());
    } else if (cmd == "CLEAR") {
        safety.clear_fault();
        Serial.println("[CMD] Fault Cleared");
    } else if (cmd == "PAGE") {
        lcdCurrentPage = (lcdCurrentPage + 1) % TOTAL_LCD_PAGES;
        renderLcdPage();
    } else if (cmd == "STATUS") {
        Serial.printf("[STATUS] State: %s | SSR: %d | Rsn: %s | 12V: %.2fV | LCD: %d\n",
                      ignition_state_to_str(safety.get_state()), safety.is_ssr_active(),
                      lock_reason_to_str(safety.get_lock_reason()), v12BatteryVolts, lcdCurrentPage);
    }
}

// ==============================================================================
// CONNECTION MANAGER (STA WITH SOFTAP FALLBACK)
// ==============================================================================

void setupNetworking() {
    WiFi.mode(WIFI_STA);
    Serial.printf("[WIFI] Connecting to Infrastructure SSID: %s...\n", wifiSsid.c_str());
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < SOFTAP_TIMEOUT_MS) {
        delay(250);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected as STA! IP: %s\n", WiFi.localIP().toString().c_str());
        softApActive = false;
    } else {
        Serial.println("\n[WIFI] STA timeout. Starting fallback SoftAP mode...");
        WiFi.mode(WIFI_AP);
        String mac = WiFi.macAddress();
        mac.replace(":", "");
        String apName = "SH-IGN-" + mac.substring(mac.length() - 4);
        WiFi.softAP(apName.c_str(), DEFAULT_SOFTAP_PASS);
        softApActive = true;
        Serial.printf("[WIFI] SoftAP Active: %s | IP: %s\n", apName.c_str(), WiFi.softAPIP().toString().c_str());
    }

    if (MDNS.begin("smarthelmet-ignition")) {
        Serial.println("[mDNS] Responder started: smarthelmet-ignition.local");
        MDNS.addService("http", "tcp", LOCAL_SERVER_PORT);
    }

    wsServer.begin();
    wsServer.onEvent(onWebSocketServerEvent);
    Serial.printf("[WS-SRV] Local WebSocket Server listening on port %d\n", LOCAL_SERVER_PORT);
}

// ==============================================================================
// ARDUINO SETUP & SUPERVISOR LOOP
// ==============================================================================

void setup() {
    // CRITICAL: Initialize SSR pin LOW immediately before anything else
    pinMode(PIN_SSR_CONTROL, OUTPUT);
    digitalWrite(PIN_SSR_CONTROL, LOW);

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_12V_SENSE, INPUT);
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);

    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH); // Default LOCKED indicator

    Serial.begin(115200);
    delay(300);
    Serial.println("\n==============================================");
    Serial.println(" SMART IGNITION CONTROLLER — FIRMWARE v1.0.0 ");
    Serial.println("==============================================");

    // Load NVS Preferences
    prefs.begin("ignition_cfg", false);
    deviceId = prefs.getString("dev_id", DEFAULT_DEVICE_ID);
    pairedHelmetId = prefs.getString("pair_id", DEFAULT_PAIRED_HELMET_ID);
    pairingPsk = prefs.getString("psk", DEFAULT_PAIRING_PSK);
    wifiSsid = prefs.getString("ssid", "SmartHelmet_AP");
    wifiPass = prefs.getString("pass", "helmet1234");
    prefs.end();

    initLcd();
    read12VBattery();
    setupNetworking();

    // Hardware Watchdog (8 seconds)
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);

    Serial.println("[SYSTEM] Ignition controller supervisor running.");
}

void loop() {
    esp_task_wdt_reset();
    wsServer.loop();

    uint32_t now = millis();

    // Periodic Analog 12V Sense (every 100ms)
    if (now - lastAnalogSenseMs >= 100) {
        lastAnalogSenseMs = now;
        read12VBattery();
        safety.update(latestHeartbeat, now, v12BatteryVolts);
        updateActuators();
        handleButtons();
        handleSerialCommands();
    }

    // Refresh LCD Display (every 500ms)
    if (now - lastLcdUpdateMs >= 500) {
        lastLcdUpdateMs = now;
        renderLcdPage();
    }
}