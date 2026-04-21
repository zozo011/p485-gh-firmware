// ============================================================
// Green Home P485 – ota.cpp
// HTTP OTA: 24h időközönként + MQTT CMD trigger
// ============================================================
#include "ota.h"
#include "config.h"
#include "logger.h"
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFi.h>

static const uint32_t OTA_CHECK_INTERVAL_MS = 24UL * 3600UL * 1000UL;
static uint32_t       _last_check           = 0;
static bool           _force_check          = false;
static char           _override_url[256]    = "";

// ============================================================
// VERZIÓ-ÖSSZEHASONLÍTÁS
// ============================================================
static bool isNewer(const char* remote, const char* local) {
    int rma = 0, rmi = 0, rpa = 0;
    int lma = 0, lmi = 0, lpa = 0;
    sscanf(remote, "%d.%d.%d", &rma, &rmi, &rpa);
    sscanf(local,  "%d.%d.%d", &lma, &lmi, &lpa);
    if (rma != lma) return rma > lma;
    if (rmi != lmi) return rmi > lmi;
    return rpa > lpa;
}

// ============================================================
// OTA DOWNLOAD + FLASH
// ============================================================
static void doFlash(const char* binUrl) {
    Serial.printf("[OTA] Flash indul: %s\n", binUrl);
    Logger::log("ota_update", "start", binUrl);

    httpUpdate.setLedPin(LED_STATUS, HIGH);
    httpUpdate.rebootOnUpdate(false);

    WiFiClient wfc;
    t_httpUpdate_return ret = httpUpdate.update(wfc, binUrl);

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            Serial.printf("[OTA] HIBA: %s\n", httpUpdate.getLastErrorString().c_str());
            Logger::log("ota_update", "fail", httpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("[OTA] Nincs frissítés.");
            break;
        case HTTP_UPDATE_OK:
            Serial.println("[OTA] Sikeres! Restart...");
            Logger::log("ota_update", "ok");
            delay(500);
            esp_restart();
            break;
    }
}

// ============================================================
// OTA CHECK – verzió fájl lekérés, ha újabb → flash
// ============================================================
static void doOtaCheck() {
    if (WiFi.status() != WL_CONNECTED) return;

    // Ha van override URL (MQTT CMD-ből) → azonnal flash
    if (_override_url[0]) {
        char url[256];
        strlcpy(url, _override_url, sizeof(url));
        _override_url[0] = 0;
        doFlash(url);
        return;
    }

    String verUrl = String(g_config.ota_url);
    if (verUrl.isEmpty() || verUrl.equals("\"\"")) {
        Serial.println("[OTA] ota_url nincs beállítva, skip.");
        return;
    }

    Serial.printf("[OTA] Verzió ellenőrzés: %s\n", verUrl.c_str());
    HTTPClient http;
    http.begin(verUrl);
    http.setTimeout(5000);
    int code = http.GET();
    if (code != 200) {
        Serial.printf("[OTA] Verzió GET hiba: %d\n", code);
        http.end();
        Logger::log("ota_check", "ver_fetch_fail");
        return;
    }

    String remote = http.getString();
    http.end();
    remote.trim();

    Serial.printf("[OTA] Remote=%s  Local=%s\n", remote.c_str(), FW_VERSION);
    Logger::log("ota_check", remote.c_str(), FW_VERSION);

    if (!isNewer(remote.c_str(), FW_VERSION)) {
        Serial.println("[OTA] Nincs újabb verzió.");
        return;
    }

    // Bin URL: ver URL mellé firmware.bin
    String binUrl = verUrl;
    int slash = binUrl.lastIndexOf('/');
    binUrl = (slash >= 0) ? binUrl.substring(0, slash + 1) + "firmware.bin"
                          : binUrl + ".bin";

    doFlash(binUrl.c_str());
}

// ============================================================
// PUBLIC API
// ============================================================
void otaSetup() {
    _last_check    = millis();  // első ellenőrzés 24h múlva
    _force_check   = false;
    _override_url[0] = 0;
    Serial.println("[OTA] HTTP OTA inicializálva.");
}

void otaLoop() {
    uint32_t now = millis();
    if (_force_check || (now - _last_check >= OTA_CHECK_INTERVAL_MS)) {
        _last_check  = now;
        _force_check = false;
        doOtaCheck();
    }
}

void otaTriggerCheck() {
    _force_check = true;
}

// MQTT CMD: {"cmd":"ota","url":"https://..."} → közvetlen bin URL
void otaTriggerWithUrl(const char* binUrl) {
    if (binUrl && binUrl[0]) {
        strlcpy(_override_url, binUrl, sizeof(_override_url));
        _force_check = true;
    }
}
