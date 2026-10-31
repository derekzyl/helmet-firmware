#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "config.h"

// ==============================================================================
// BLE GATT PROVISIONING (SH-IGN-PROV) — PROTOCOL.md §5
// Status: 0=IDLE 1=APPLYING 2=SUCCESS 3=FAILED
// ==============================================================================

namespace ble_prov {

enum Status : uint8_t {
    IDLE = 0,
    APPLYING = 1,
    SUCCESS = 2,
    FAILED = 3
};

inline bool active = false;
inline volatile bool applyRequested = false;
inline volatile bool clientConnected = false;
inline Status status = IDLE;

inline String pendingSsid;
inline String pendingPass;
inline String pendingPsk;
inline String pendingPairId;

inline BLEServer* server = nullptr;
inline BLECharacteristic* statusChar = nullptr;
inline const char* currentAdvName = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override { clientConnected = true; }
    void onDisconnect(BLEServer* s) override {
        clientConnected = false;
        // Do not re-advertise after SUCCESS — phone must not hammer a dying peripheral.
        if (active && status != SUCCESS) {
            s->startAdvertising();
        }
    }
};

class WriteCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        std::string v = c->getValue();
        if (v.empty()) return;
        String uuid = String(c->getUUID().toString().c_str());
        String val = String(v.c_str());
        val.trim();

        if (uuid.equalsIgnoreCase(BLE_CHAR_SSID_UUID)) {
            pendingSsid = val;
        } else if (uuid.equalsIgnoreCase(BLE_CHAR_PASS_UUID)) {
            pendingPass = val;
        } else if (uuid.equalsIgnoreCase(BLE_CHAR_PSK_UUID)) {
            pendingPsk = val;
        } else if (uuid.equalsIgnoreCase(BLE_CHAR_PAIRID_UUID)) {
            pendingPairId = val;
        } else if (uuid.equalsIgnoreCase(BLE_CHAR_STATUS_UUID)) {
            if (v[0] == 1 || val == "1") {
                applyRequested = true;
            }
        }
    }
};

inline void setStatus(Status s) {
    status = s;
    if (statusChar) {
        uint8_t b = static_cast<uint8_t>(s);
        statusChar->setValue(&b, 1);
        statusChar->notify();
    }
}

inline void stopAdvertising() {
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    if (adv) adv->stop();
}

inline void start(const char* advName) {
    if (active) return;
    active = true;
    applyRequested = false;
    status = IDLE;
    pendingSsid = "";
    pendingPass = "";
    pendingPsk = "";
    pendingPairId = "";
    currentAdvName = advName;

    BLEDevice::init(advName);
    BLEDevice::setPower(ESP_PWR_LVL_P6);

    server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    BLEService* svc = server->createService(BLE_SERVICE_UUID);
    auto* writeCb = new WriteCallbacks();

    auto* ssid = svc->createCharacteristic(BLE_CHAR_SSID_UUID, BLECharacteristic::PROPERTY_WRITE);
    auto* pass = svc->createCharacteristic(BLE_CHAR_PASS_UUID, BLECharacteristic::PROPERTY_WRITE);
    auto* psk = svc->createCharacteristic(BLE_CHAR_PSK_UUID, BLECharacteristic::PROPERTY_WRITE);
    auto* pair = svc->createCharacteristic(BLE_CHAR_PAIRID_UUID, BLECharacteristic::PROPERTY_WRITE);
    statusChar = svc->createCharacteristic(
        BLE_CHAR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    statusChar->addDescriptor(new BLE2902());

    ssid->setCallbacks(writeCb);
    pass->setCallbacks(writeCb);
    psk->setCallbacks(writeCb);
    pair->setCallbacks(writeCb);
    statusChar->setCallbacks(writeCb);

    uint8_t idle = IDLE;
    statusChar->setValue(&idle, 1);

    svc->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    BLEAdvertisementData advData;
    advData.setName(advName);
    advData.setCompleteServices(BLEUUID(BLE_SERVICE_UUID));
    adv->setAdvertisementData(advData);

    BLEAdvertisementData scanData;
    scanData.setName(advName);
    adv->setScanResponseData(scanData);

    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.printf("[BLE] Advertising as %s (FE20 in adv+scanRsp)\n", advName);
}

inline void stop() {
    if (!active) return;
    stopAdvertising();
    delay(40);
    BLEDevice::deinit(true);
    server = nullptr;
    statusChar = nullptr;
    currentAdvName = nullptr;
    active = false;
    clientConnected = false;
    Serial.println("[BLE] Provisioning stopped");
}

/** Persist any pending fields that were written; returns true if reboot needed. */
inline bool applyToNvs(Preferences& prefs, const char* ns,
                       String& wifiSsid, String& wifiPass,
                       String& pairingPsk, String& pairedHelmetId) {
    if (!applyRequested) return false;
    applyRequested = false;
    setStatus(APPLYING);

    bool ok = false;
    prefs.begin(ns, false);
    if (pendingSsid.length() > 0) {
        wifiSsid = pendingSsid;
        prefs.putString("ssid", wifiSsid);
        ok = true;
    }
    if (pendingPass.length() > 0) {
        wifiPass = pendingPass;
        prefs.putString("pass", wifiPass);
        ok = true;
    }
    if (pendingPsk.length() >= 16) {
        pairingPsk = pendingPsk;
        prefs.putString("psk", pairingPsk);
        ok = true;
    }
    if (pendingPairId.length() > 0) {
        pairedHelmetId = pendingPairId;
        prefs.putString("pair_id", pairedHelmetId);
        ok = true;
    }
    if (ok) {
        prefs.putBool("provisioned", true);
        prefs.putBool("auto_connect", true);
        bool softOnly = (wifiSsid.length() == 0 || wifiSsid == SOFTAP_ONLY_SSID);
        prefs.putBool("prefer_softap", softOnly);
    }
    prefs.end();

    if (ok) {
        setStatus(SUCCESS);
        stopAdvertising();
        Serial.println("[BLE] Credentials saved — advertising stopped, rebooting");
        delay(500);
        return true;
    }
    setStatus(FAILED);
    Serial.println("[BLE] Apply failed: missing credentials");
    return false;
}

}  // namespace ble_prov
