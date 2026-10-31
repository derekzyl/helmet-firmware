#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <esp_task_wdt.h>
#include <cstring>
#include <cstdlib>

#include "pins.h"
#include "config.h"
#include "crypto_utils.h"
#include "safety_fsm.h"
#include "ble_provision.h"

// SoftAP heartbeat may carry numbers or legacy stringified floats
static float jsonAsFloat(JsonVariantConst v, float fallback = 0.0f) {
    if (v.isNull()) return fallback;
    if (v.is<float>() || v.is<double>() || v.is<int>() || v.is<long>()) return v.as<float>();
    if (v.is<const char*>()) {
        const char* s = v.as<const char*>();
        if (!s || !*s) return fallback;
        return static_cast<float>(atof(s));
    }
    return fallback;
}

static double jsonAsDouble(JsonVariantConst v, double fallback = 0.0) {
    if (v.isNull()) return fallback;
    if (v.is<float>() || v.is<double>() || v.is<int>() || v.is<long>()) return v.as<double>();
    if (v.is<const char*>()) {
        const char* s = v.as<const char*>();
        if (!s || !*s) return fallback;
        return atof(s);
    }
    return fallback;
}

// ==============================================================================
// GLOBAL SYSTEM INSTANCES & STATE
// ==============================================================================

Preferences prefs;
WebSocketsServer wsServer(LOCAL_SERVER_PORT);
WebServer httpServer(80);  // phone/app SoftAP status (live sensors)
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);

SafetyFSM safety;
HeartbeatInput latestHeartbeat;

String deviceId = DEFAULT_DEVICE_ID;
String pairedHelmetId = DEFAULT_PAIRED_HELMET_ID;
String pairingPsk = DEFAULT_PAIRING_PSK;
String wifiSsid = DEFAULT_WIFI_SSID;
String wifiPass = DEFAULT_WIFI_PASS;
String softApSsid = "";
bool autoConnect = true;
bool preferSoftAp = true;  // SoftAP peer mode — no phone Wi‑Fi required after pairing
bool provisioned = true;  // factory defaults enable out-of-box auto-connect

float v12BatteryVolts = 12.6f;
uint8_t lcdCurrentPage = 0;
bool lcdDetected = false;
bool softApActive = false;
bool helmetClientLinked = false;
uint32_t lastSeqReceived = 0;
uint32_t lastLcdUpdateMs = 0;
uint32_t lastAnalogSenseMs = 0;
uint32_t lastLedMs = 0;
uint32_t btn1PressStartMs = 0;
uint32_t btn2PressStartMs = 0;
uint32_t softApStartedMs = 0;
uint32_t wifiConnectStartedMs = 0;
uint32_t lastCloudForwardMs = 0;
uint32_t lastHelmetHbMs = 0;
bool btn2ProvArmed = false;

enum class NetLedMode : uint8_t {
    SAFETY,       // normal locked/unlocked LEDs
    WIFI_TRY,     // red slow blink — STA connecting
    SOFTAP_WAIT,  // green slow blink — waiting for helmet
    BLE_PROV,     // green/red alternate — app pairing
    LINK_FAIL     // red fast blink
};

NetLedMode netLedMode = NetLedMode::WIFI_TRY;
bool ledPhase = false;

// ==============================================================================
// HARDWARE ACTUATION & I/O
// ==============================================================================

void updateActuators() {
    bool ssrActive = safety.is_ssr_active();
    digitalWrite(PIN_SSR_CONTROL, ssrActive ? HIGH : LOW);

    uint32_t now = millis();
    uint16_t period = LINK_LED_BLINK_SLOW_MS;

    switch (netLedMode) {
        case NetLedMode::BLE_PROV:
            period = LINK_LED_BLINK_FAST_MS;
            if (now - lastLedMs >= period) {
                lastLedMs = now;
                ledPhase = !ledPhase;
                digitalWrite(PIN_LED_GREEN, ledPhase ? HIGH : LOW);
                digitalWrite(PIN_LED_RED, ledPhase ? LOW : HIGH);
            }
            break;

        case NetLedMode::WIFI_TRY:
            period = LINK_LED_BLINK_SLOW_MS;
            if (now - lastLedMs >= period) {
                lastLedMs = now;
                ledPhase = !ledPhase;
                digitalWrite(PIN_LED_GREEN, LOW);
                digitalWrite(PIN_LED_RED, ledPhase ? HIGH : LOW);
            }
            break;

        case NetLedMode::SOFTAP_WAIT:
            period = LINK_LED_BLINK_SLOW_MS;
            if (now - lastLedMs >= period) {
                lastLedMs = now;
                ledPhase = !ledPhase;
                digitalWrite(PIN_LED_GREEN, ledPhase ? HIGH : LOW);
                digitalWrite(PIN_LED_RED, LOW);
            }
            break;

        case NetLedMode::LINK_FAIL:
            period = LINK_LED_BLINK_FAST_MS;
            if (now - lastLedMs >= period) {
                lastLedMs = now;
                ledPhase = !ledPhase;
                digitalWrite(PIN_LED_GREEN, LOW);
                digitalWrite(PIN_LED_RED, ledPhase ? HIGH : LOW);
            }
            break;

        case NetLedMode::SAFETY:
        default:
            // GREEN = helmet connected (peer SoftAP / WS heartbeat).
            // RED only when NO helmet link (or FAULT). Locked-but-linked stays GREEN.
            if (safety.get_state() == IgnitionState::FAULT) {
                digitalWrite(PIN_LED_GREEN, LOW);
                digitalWrite(PIN_LED_RED, HIGH);
            } else if (helmetClientLinked) {
                digitalWrite(PIN_LED_GREEN, HIGH);
                digitalWrite(PIN_LED_RED, LOW);
            } else {
                period = LINK_LED_BLINK_FAST_MS;
                if (now - lastLedMs >= period) {
                    lastLedMs = now;
                    ledPhase = !ledPhase;
                    digitalWrite(PIN_LED_GREEN, LOW);
                    digitalWrite(PIN_LED_RED, ledPhase ? HIGH : LOW);
                }
            }
            break;
    }

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

void refreshNetLedMode() {
    if (ble_prov::active) {
        netLedMode = NetLedMode::BLE_PROV;
        return;
    }
    if (WiFi.status() != WL_CONNECTED && !softApActive) {
        netLedMode = NetLedMode::WIFI_TRY;
        return;
    }
    if (!helmetClientLinked) {
        netLedMode = softApActive ? NetLedMode::SOFTAP_WAIT : NetLedMode::SAFETY;
        return;
    }
    // Helmet heartbeat alive — solid green via SAFETY branch
    netLedMode = NetLedMode::SAFETY;
}

// ==============================================================================
// 12V MOTORCYCLE BATTERY SENSE
// ==============================================================================

void read12VBattery() {
    uint16_t rawAdc = analogRead(PIN_12V_SENSE);
    float vAdc = (rawAdc / 4095.0f) * 3.3f;
    float vActual = vAdc * ADC_12V_DIVIDER_RATIO;
    if (vActual < 2.0f) vActual = 12.6f;
    v12BatteryVolts = vActual;
}

// ==============================================================================
// 16x2 LCD DISPLAY (OPTIONAL — soft-fails if backpack missing)
// ==============================================================================

void initLcd() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
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
        Serial.println("[LCD] Not detected — app + LEDs are primary UI");
    }
}

void renderLcdPage() {
    if (!lcdDetected) return;

    lcd.clear();
    switch (lcdCurrentPage) {
        case 0:
            lcd.setCursor(0, 0);
            lcd.printf("ST:%s", ignition_state_to_str(safety.get_state()));
            lcd.setCursor(0, 1);
            if (safety.is_ssr_active()) {
                lcd.print("SSR:ENGAGED (ON)");
            } else {
                lcd.printf("RSN:%s", lock_reason_to_str(safety.get_lock_reason()));
            }
            break;

        case 1:
            lcd.setCursor(0, 0);
            if (ble_prov::active) {
                lcd.print("NET:BLE PAIRING");
            } else if (softApActive) {
                lcd.print("NET:SoftAP MODE");
            } else if (WiFi.status() == WL_CONNECTED) {
                lcd.printf("SSID:%s", WiFi.SSID().substring(0, 10).c_str());
            } else {
                lcd.print("NET:CONNECTING");
            }
            lcd.setCursor(0, 1);
            if (softApActive) {
                lcd.printf("%s", softApSsid.substring(0, 16).c_str());
            } else {
                lcd.printf("IP:%s", WiFi.localIP().toString().c_str());
            }
            break;

        case 2:
            lcd.setCursor(0, 0);
            lcd.printf("HLM BATT: %d%%", latestHeartbeat.batt_pct);
            lcd.setCursor(0, 1);
            lcd.printf("BAC: %.3f%% %s", latestHeartbeat.bac,
                       (latestHeartbeat.bac > 0.05f) ? "FAIL" : "PASS");
            break;

        case 3:
            lcd.setCursor(0, 0);
            lcd.print("BIKE 12V BATTERY");
            lcd.setCursor(0, 1);
            lcd.printf("VOLTS: %.2fV %s", v12BatteryVolts,
                       (v12BatteryVolts < V12_CRITICAL_LOW_VOLTS) ? "LOW" : "OK");
            break;

        case 4:
            lcd.setCursor(0, 0);
            lcd.printf("UP: %lus", millis() / 1000);
            lcd.setCursor(0, 1);
            lcd.printf("LINK:%s #%lu", helmetClientLinked ? "OK" : "--", lastSeqReceived);
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
            if (wsServer.connectedClients() == 0) {
                helmetClientLinked = false;
            }
            break;

        case WStype_CONNECTED:
            Serial.printf("[WS-SRV] Client #%u Connected\n", num);
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

            if (String(senderDev) != pairedHelmetId) {
                Serial.printf("[WS-SRV] REJECTED: Unpaired device %s (expected %s)\n",
                              senderDev, pairedHelmetId.c_str());
                return;
            }

            if (seq <= lastSeqReceived && lastSeqReceived > 0) {
                Serial.printf("[WS-SRV] REJECTED: Replay seq=%lu <= last=%lu\n",
                              seq, lastSeqReceived);
                return;
            }

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

            lastSeqReceived = seq;
            helmetClientLinked = true;
            safety.register_heartbeat(millis());

            if (strcmp(msgType, "HEARTBEAT") == 0) {
                latestHeartbeat.worn = data["worn"] | false;
                latestHeartbeat.worn_ms = data["worn_ms"] | 0;
                latestHeartbeat.bac = jsonAsFloat(data["bac"], 0.0f);
                latestHeartbeat.mq3_warm = data["mq3_warm"] | false;
                latestHeartbeat.impact = data["impact"] | false;
                latestHeartbeat.batt_pct = data["batt_pct"] | 100;
                latestHeartbeat.speed_kmh = jsonAsFloat(data["speed"], 0.0f);
                latestHeartbeat.ir_near = data["ir_near"] | latestHeartbeat.worn;
                latestHeartbeat.ir_raw = data["ir_raw"] | 1;
                latestHeartbeat.mq3_raw = data["mq3_raw"] | 0;
                latestHeartbeat.tilt_deg = jsonAsFloat(data["tilt_deg"], 0.0f);
                latestHeartbeat.impact_g = jsonAsFloat(data["impact_g"], 1.0f);
                latestHeartbeat.ax = jsonAsFloat(data["ax"], 0.0f);
                latestHeartbeat.ay = jsonAsFloat(data["ay"], 0.0f);
                latestHeartbeat.az = jsonAsFloat(data["az"], 1.0f);
                latestHeartbeat.imu_anomaly = data["imu_anomaly"] | false;
                latestHeartbeat.tilt_warn = data["tilt_warn"] | false;
                latestHeartbeat.mpu_ok = data["mpu_ok"] | false;
                latestHeartbeat.gps_fix = data["gps_fix"] | false;
                latestHeartbeat.latitude = jsonAsDouble(data["lat"], 0.0);
                latestHeartbeat.longitude = jsonAsDouble(data["lon"], 0.0);
                {
                    const char* imu = data["imu_state"] | "IDLE";
                    strncpy(latestHeartbeat.imu_state, imu, sizeof(latestHeartbeat.imu_state) - 1);
                    latestHeartbeat.imu_state[sizeof(latestHeartbeat.imu_state) - 1] = '\0';
                }
                lastHelmetHbMs = millis();

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
                Serial.printf("[WS-SRV] Maintenance Override for %lu s\n", duration);
            }
            break;
        }

        default:
            break;
    }
}

// ==============================================================================
// BLE PROVISIONING ENTRY
// ==============================================================================

void enterBleProvisioning(bool keepSoftAp = false) {
    if (ble_prov::active) return;
    Serial.println("[BLE] Entering app provisioning mode (SH-IGN-PROV)");
    if (!keepSoftAp) {
        wsServer.close();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(200);
        softApActive = false;
        helmetClientLinked = false;
    } else {
        // SoftAP stays for helmet peer, but drop STA uplink during BLE.
        // AP+STA+BLE coexistence is a common cause of Android GATT status 133.
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[BLE] Dropping home STA uplink during provision (SoftAP kept)");
            WiFi.disconnect(false);
        }
        WiFi.mode(WIFI_AP);
        WiFi.setSleep(true);
        delay(100);
    }
    ble_prov::start(BLE_ADV_NAME);
    netLedMode = NetLedMode::BLE_PROV;
}

// ==============================================================================
// BUTTON INPUT HANDLING
// ==============================================================================

void handleButtons() {
    uint32_t now = millis();

    if (digitalRead(PIN_BUTTON_1) == LOW) {
        if (btn1PressStartMs == 0) btn1PressStartMs = now;
    } else {
        if (btn1PressStartMs > 0) {
            uint32_t duration = now - btn1PressStartMs;
            if (duration >= 50 && duration < 2000) {
                lcdCurrentPage = (lcdCurrentPage + 1) % TOTAL_LCD_PAGES;
                renderLcdPage();
                Serial.printf("[BUTTON-1] LCD Page %d\n", lcdCurrentPage);
            }
            btn1PressStartMs = 0;
        }
    }

    if (digitalRead(PIN_BUTTON_2) == LOW) {
        if (btn2PressStartMs == 0) {
            btn2PressStartMs = now;
            btn2ProvArmed = false;
        } else if (now - btn2PressStartMs >= 15000) {
            Serial.println("[BUTTON-2] 15s Hold: FACTORY RESET!");
            prefs.begin("ignition_cfg", false);
            prefs.clear();
            prefs.end();
            btn2PressStartMs = 0;
            ESP.restart();
        } else if (!btn2ProvArmed && now - btn2PressStartMs >= 5000) {
            btn2ProvArmed = true;
            Serial.println("[BUTTON-2] 5s Hold: BLE Provisioning");
            enterBleProvisioning();
        }
    } else {
        btn2PressStartMs = 0;
        btn2ProvArmed = false;
    }
}

// ==============================================================================
// SERIAL COMMAND CLI
// ==============================================================================

void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "UNLOCK") {
        Serial.println("[CMD] Simulating Safe Rider Unlock");
        latestHeartbeat.worn = true;
        latestHeartbeat.worn_ms = 2000;
        latestHeartbeat.mq3_warm = true;
        latestHeartbeat.bac = 0.00f;
        latestHeartbeat.batt_pct = 95;
        helmetClientLinked = true;
        safety.register_heartbeat(millis());
    } else if (cmd == "ALCOHOL") {
        latestHeartbeat.bac = 0.09f;
        safety.register_heartbeat(millis());
    } else if (cmd == "CRASH") {
        latestHeartbeat.impact = true;
        safety.register_heartbeat(millis());
    } else if (cmd == "CLEAR") {
        safety.clear_fault();
    } else if (cmd == "PROV" || cmd == "BLE") {
        enterBleProvisioning();
    } else if (cmd == "PAGE") {
        lcdCurrentPage = (lcdCurrentPage + 1) % TOTAL_LCD_PAGES;
        renderLcdPage();
    } else if (cmd == "STATUS") {
        Serial.printf("[STATUS] State:%s SSR:%d Rsn:%s 12V:%.2f SoftAP:%s BLE:%d Link:%d\n",
                      ignition_state_to_str(safety.get_state()), safety.is_ssr_active(),
                      lock_reason_to_str(safety.get_lock_reason()), v12BatteryVolts,
                      softApSsid.c_str(), ble_prov::active, helmetClientLinked);
    }
}

// ==============================================================================
// CONNECTION MANAGER (STA → SoftAP fallback → optional BLE)
// ==============================================================================

void handleHttpStatus() {
    StaticJsonDocument<768> doc;
    doc["device_id"] = pairedHelmetId;
    doc["ignition_id"] = deviceId;
    doc["helmet_worn"] = latestHeartbeat.worn;
    doc["ir_near"] = latestHeartbeat.ir_near;
    doc["ir_raw"] = latestHeartbeat.ir_raw;
    doc["bac_level"] = latestHeartbeat.bac;
    doc["alcohol_detected"] = latestHeartbeat.bac > 0.05f;
    doc["mq3_raw"] = latestHeartbeat.mq3_raw;
    doc["mq3_warm"] = latestHeartbeat.mq3_warm;
    doc["mq3_warmed_up"] = latestHeartbeat.mq3_warm;
    doc["battery_percentage"] = latestHeartbeat.batt_pct;
    doc["battery_pct"] = latestHeartbeat.batt_pct;
    doc["impact_g"] = latestHeartbeat.impact_g;
    doc["tilt_degrees"] = latestHeartbeat.tilt_deg;
    doc["tilt_deg"] = latestHeartbeat.tilt_deg;
    doc["tilt_warning"] = latestHeartbeat.tilt_warn;
    doc["ax"] = latestHeartbeat.ax;
    doc["ay"] = latestHeartbeat.ay;
    doc["az"] = latestHeartbeat.az;
    doc["imu_state"] = latestHeartbeat.imu_state;
    doc["imu_anomaly"] = latestHeartbeat.imu_anomaly;
    doc["mpu_ok"] = latestHeartbeat.mpu_ok;
    doc["speed_kmh"] = latestHeartbeat.speed_kmh;
    doc["gps_fixed"] = latestHeartbeat.gps_fix;
    doc["latitude"] = latestHeartbeat.latitude;
    doc["longitude"] = latestHeartbeat.longitude;
    doc["ignition_status"] = ignition_state_to_str(safety.get_state());
    doc["ignition_state"] = ignition_state_to_str(safety.get_state());
    doc["lock_reason"] = lock_reason_to_str(safety.get_lock_reason());
    doc["local_link_connected"] = helmetClientLinked;
    doc["wifi_mode"] = softApActive ? "SOFTAP" : "STA";
    doc["wifi_ssid"] = softApActive ? softApSsid : wifiSsid;
    doc["internet_ok"] = (WiFi.status() == WL_CONNECTED);
    doc["shared_wifi"] = (WiFi.status() == WL_CONNECTED);
    doc["prefer_softap"] = preferSoftAp;
    doc["live_local"] = true;
    doc["helmet_connected"] = helmetClientLinked;
    doc["ignition_connected"] = true;
    doc["both_linked"] = helmetClientLinked;
    doc["local_link_connected"] = helmetClientLinked;
    doc["hb_age_ms"] = lastHelmetHbMs == 0 ? -1 : (int)(millis() - lastHelmetHbMs);
    doc["timestamp"] = (uint32_t)(millis() / 1000);

    String out;
    serializeJson(doc, out);
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.send(200, "application/json", out);
}

void startHttpStatusServer() {
    httpServer.on("/status", HTTP_GET, handleHttpStatus);
    httpServer.on("/", HTTP_GET, handleHttpStatus);
    httpServer.begin();
    Serial.println("[HTTP] SoftAP status on :80/status (live helmet sensors)");
}

void forwardHelmetTelemetryToCloud() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (lastHelmetHbMs == 0) return;
    if (millis() - lastCloudForwardMs < CLOUD_FORWARD_INTERVAL_MS) return;
    lastCloudForwardMs = millis();

    HTTPClient http;
    String url = String("https://") + CLOUD_BACKEND_HOST + CLOUD_TELEMETRY_ENDPOINT;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<768> doc;
    doc["device_id"] = pairedHelmetId;
    doc["helmet_worn"] = latestHeartbeat.worn;
    doc["bac_level"] = latestHeartbeat.bac;
    doc["alcohol_detected"] = latestHeartbeat.bac > 0.05f;
    doc["mq3_raw"] = latestHeartbeat.mq3_raw;
    doc["battery_percentage"] = latestHeartbeat.batt_pct;
    doc["impact_g"] = latestHeartbeat.impact_g;
    doc["tilt_degrees"] = latestHeartbeat.tilt_deg;
    doc["tilt_warning"] = latestHeartbeat.tilt_warn;
    doc["ax"] = latestHeartbeat.ax;
    doc["ay"] = latestHeartbeat.ay;
    doc["az"] = latestHeartbeat.az;
    doc["imu_state"] = latestHeartbeat.imu_state;
    doc["imu_anomaly"] = latestHeartbeat.imu_anomaly;
    doc["mpu_ok"] = latestHeartbeat.mpu_ok;
    doc["speed_kmh"] = latestHeartbeat.speed_kmh;
    doc["gps_fixed"] = latestHeartbeat.gps_fix;
    doc["latitude"] = latestHeartbeat.latitude;
    doc["longitude"] = latestHeartbeat.longitude;
    doc["ignition_status"] = ignition_state_to_str(safety.get_state());
    doc["lock_reason"] = lock_reason_to_str(safety.get_lock_reason());
    doc["local_link_connected"] = helmetClientLinked;
    doc["wifi_mode"] = softApActive ? "APSTA" : "STA";
    doc["wifi_ssid"] = wifiSsid;
    doc["internet_ok"] = true;
    doc["shared_wifi"] = true;
    doc["prefer_softap"] = true;

    String body;
    serializeJson(doc, body);
    int code = http.POST(body);
    http.end();
    if (code > 0) {
        Serial.printf("[CLOUD] Forwarded helmet telemetry HTTP %d\n", code);
    }
}

bool isSoftApOnlySsid(const String& ssid) {
    return ssid.length() == 0 || ssid == SOFTAP_ONLY_SSID;
}

// SoftAP is ALWAYS on for helmet↔ignition offline peer.
// Home Wi‑Fi (STA) is optional uplink so the phone can stay on home Wi‑Fi
// while ignition forwards helmet sensors to the cloud.
void startSoftAp(bool withStaUplink) {
    WiFi.mode(withStaUplink ? WIFI_AP_STA : WIFI_AP);
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    softApSsid = String(DEFAULT_SOFTAP_PREFIX) + mac.substring(mac.length() - 4);
    WiFi.softAP(softApSsid.c_str(), DEFAULT_SOFTAP_PASS);
    softApActive = true;
    softApStartedMs = millis();
    Serial.printf("[WIFI] SoftAP peer Active: %s | IP: %s | pass: %s | APSTA=%d\n",
                  softApSsid.c_str(), WiFi.softAPIP().toString().c_str(), DEFAULT_SOFTAP_PASS,
                  withStaUplink ? 1 : 0);

    prefs.begin("ignition_cfg", false);
    prefs.putString("softap_ssid", softApSsid);
    prefs.end();
}

bool connectHomeWifiSta() {
    if (isSoftApOnlySsid(wifiSsid) || !autoConnect) return false;
    netLedMode = NetLedMode::WIFI_TRY;
    Serial.printf("[WIFI] Home STA uplink → %s (phone stays on this Wi‑Fi)\n", wifiSsid.c_str());
    wifiConnectStartedMs = millis();
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < SOFTAP_TIMEOUT_MS) {
        delay(200);
        updateActuators();
        Serial.print(".");
        esp_task_wdt_reset();
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] STA OK IP: %s — cloud forward enabled\n", WiFi.localIP().toString().c_str());
        return true;
    }
    Serial.println("\n[WIFI] STA timeout — SoftAP peer still works offline");
    return false;
}

void setupNetworking() {
    // Peer SoftAP first (helmet never needs home Wi‑Fi)
    const bool wantSta = autoConnect && !isSoftApOnlySsid(wifiSsid);
    startSoftAp(wantSta);

    if (wantSta) {
        connectHomeWifiSta();
    } else {
        Serial.println("[WIFI] SoftAP-only peer (no home uplink) — sensors stay local until home Wi‑Fi is provisioned");
    }

    if (!autoConnect) {
        enterBleProvisioning(true);
    }

    if (MDNS.begin("smarthelmet-ignition")) {
        Serial.println("[mDNS] smarthelmet-ignition.local");
        MDNS.addService("http", "tcp", LOCAL_SERVER_PORT);
    }

    wsServer.begin();
    wsServer.onEvent(onWebSocketServerEvent);
    startHttpStatusServer();
    Serial.printf("[WS-SRV] Listening SoftAP :%d + HTTP :80\n", LOCAL_SERVER_PORT);
}

void maintainProvisioning() {
    if (!ble_prov::active) {
        // After SoftAP is up with no helmet, open BLE so app can pair without LCD/button
        if (softApActive && !helmetClientLinked &&
            softApStartedMs > 0 &&
            (millis() - softApStartedMs) >= AUTO_BLE_AFTER_NO_CLIENT_MS) {
            Serial.println("[BLE] No helmet link — auto-opening BLE (SoftAP stays up)");
            enterBleProvisioning(true);
            softApStartedMs = 0;  // only auto-open once per SoftAP session
        }
        return;
    }

    if (ble_prov::applyToNvs(prefs, "ignition_cfg", wifiSsid, wifiPass, pairingPsk, pairedHelmetId)) {
        ESP.restart();
    }
}

// ==============================================================================
// ARDUINO SETUP & SUPERVISOR LOOP
// ==============================================================================

void setup() {
    pinMode(PIN_SSR_CONTROL, OUTPUT);
    digitalWrite(PIN_SSR_CONTROL, LOW);

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_12V_SENSE, INPUT);
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);

    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);

    Serial.begin(115200);
    delay(300);
    Serial.println("\n==============================================");
    Serial.println(" SMART IGNITION CONTROLLER — FIRMWARE v1.1.0 ");
    Serial.println(" LCD optional | App BLE pairing + LED status  ");
    Serial.println("==============================================");

    prefs.begin("ignition_cfg", false);
    deviceId = prefs.getString("dev_id", DEFAULT_DEVICE_ID);
    pairedHelmetId = prefs.getString("pair_id", DEFAULT_PAIRED_HELMET_ID);
    pairingPsk = prefs.getString("psk", DEFAULT_PAIRING_PSK);
    wifiSsid = prefs.getString("ssid", SOFTAP_ONLY_SSID);
    wifiPass = prefs.getString("pass", DEFAULT_SOFTAP_PASS);
    softApSsid = prefs.getString("softap_ssid", "");
    autoConnect = prefs.getBool("auto_connect", true);
    preferSoftAp = prefs.getBool("prefer_softap", true);
    provisioned = prefs.getBool("provisioned", true);
    if (wifiSsid == SOFTAP_ONLY_SSID || wifiSsid.length() == 0) preferSoftAp = true;
    prefs.end();

    initLcd();
    read12VBattery();
    setupNetworking();

    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);

    Serial.printf("[SYSTEM] Device=%s PairedHelmet=%s AutoConnect=%d\n",
                  deviceId.c_str(), pairedHelmetId.c_str(), autoConnect);
}

void loop() {
    esp_task_wdt_reset();
    if (!ble_prov::active) {
        wsServer.loop();
    }
    httpServer.handleClient();

    uint32_t now = millis();

    if (now - lastAnalogSenseMs >= 100) {
        lastAnalogSenseMs = now;
        read12VBattery();
        safety.update(latestHeartbeat, now, v12BatteryVolts);
        refreshNetLedMode();
        updateActuators();
        handleButtons();
        handleSerialCommands();
        maintainProvisioning();
        forwardHelmetTelemetryToCloud();
    }

    if (now - lastLcdUpdateMs >= 500) {
        lastLcdUpdateMs = now;
        renderLcdPage();
    }
}
