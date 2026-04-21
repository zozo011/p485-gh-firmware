// ============================================================
// Green Home P485 – ota_guard.cpp
// ============================================================
#include "ota_guard.h"
#include "logger.h"
#include <Preferences.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

static Preferences s_prefs;
static bool        s_marked_good     = false;
static uint32_t    s_stable_start_ms = 0;
static uint32_t    s_boot_count      = 0;
static bool        s_pending         = true;

static const char* NS  = "otaguard";
static const char* KEY = "bootcnt";

static const esp_partition_t* _running() {
    return esp_ota_get_running_partition();
}
static const esp_partition_t* _nextBoot() {
    return esp_ota_get_next_update_partition(nullptr);
}

const char* otaGuardPartitionLabel() {
    const esp_partition_t* p = _running();
    return p ? p->label : "unknown";
}

uint32_t otaGuardBootCount() { return s_boot_count; }
bool otaGuardIsPending()     { return s_pending; }

// ------------------------------------------------------------
// BEGIN – setup() legelso lepeseinek egyike!
// ------------------------------------------------------------
void otaGuardBegin() {
    s_prefs.begin(NS, false);
    s_boot_count = s_prefs.getUInt(KEY, 0) + 1;
    s_prefs.putUInt(KEY, s_boot_count);
    s_prefs.end();

    Serial.printf("[GUARD] Running partition: %s, boot_count=%u\n",
                  otaGuardPartitionLabel(), s_boot_count);

    // Ha tul sokszor bootoltunk anelkul hogy stabil lett volna -> rollback
    if (s_boot_count > GUARD_MAX_BOOTS_BEFORE_ROLLBACK) {
        Serial.printf("[GUARD] Boot-loop detected (%u > %d). ROLLBACK...\n",
                      s_boot_count, GUARD_MAX_BOOTS_BEFORE_ROLLBACK);
        const esp_partition_t* next = _nextBoot();
        if (next) {
            // boot szamlalo nullazas a kovetkezo image kedveert
            s_prefs.begin(NS, false);
            s_prefs.putUInt(KEY, 0);
            s_prefs.end();

            esp_err_t err = esp_ota_set_boot_partition(next);
            Serial.printf("[GUARD] Set boot -> %s err=0x%x. Restart.\n",
                          next->label, err);
            delay(200);
            esp_restart();
        } else {
            Serial.println("[GUARD] Nincs masik OTA slot, skip rollback.");
        }
    }

    // Beallitjuk a pending allapotot ESP-IDF szerint (ha a bootloader
    // tamogatja). Arduino-ESP32-ben altalaban nem pending, de nem art.
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(_running(), &st) == ESP_OK) {
        s_pending = (st == ESP_OTA_IMG_PENDING_VERIFY);
        Serial.printf("[GUARD] IDF state=%d pending=%d\n", st, s_pending);
    }

    s_stable_start_ms = millis();
}

// ------------------------------------------------------------
// MARK GOOD – torli a boot-szamlalot, s esp_ota_mark_app_valid-ot hiv
// ------------------------------------------------------------
bool otaGuardMarkGood() {
    if (s_marked_good) return true;

    // 1) NVS boot-szamlalo nullazas
    s_prefs.begin(NS, false);
    s_prefs.putUInt(KEY, 0);
    s_prefs.end();

    // 2) ESP-IDF oldal (ha pending)
    esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
    Serial.printf("[GUARD] Mark-good err=0x%x. Boot-counter reset.\n", e);
    Logger::log("ota_guard", "mark_good");

    s_marked_good = true;
    s_pending     = false;
    return true;
}

// ------------------------------------------------------------
// LOOP – ha N mp-ig stabil minden, mark-good
// ------------------------------------------------------------
void otaGuardLoop() {
    if (s_marked_good) return;

    bool stable = (WiFi.status() == WL_CONNECTED);
    if (!stable) {
        s_stable_start_ms = millis();
        return;
    }

    if (millis() - s_stable_start_ms >= GUARD_STABLE_MS) {
        otaGuardMarkGood();
    }
}

// ------------------------------------------------------------
// FORCE ROLLBACK – manualis (HTTP/MQTT trigger)
// ------------------------------------------------------------
bool otaGuardForceRollback() {
    const esp_partition_t* next = _nextBoot();
    if (!next) return false;

    s_prefs.begin(NS, false);
    s_prefs.putUInt(KEY, 0);
    s_prefs.end();

    Serial.printf("[GUARD] Manual rollback -> %s\n", next->label);
    Logger::log("ota_guard", "manual_rollback", next->label);

    esp_err_t e = esp_ota_set_boot_partition(next);
    if (e != ESP_OK) return false;

    delay(500);
    esp_restart();
    return true; // sose ide er
}

void otaGuardStatus(JsonDocument& doc) {
    const esp_partition_t* run  = _running();
    const esp_partition_t* next = _nextBoot();

    doc["running"]    = run  ? run->label  : "?";
    doc["next"]       = next ? next->label : "?";
    doc["boot_count"] = s_boot_count;
    doc["marked_good"]= s_marked_good;
    doc["pending"]    = s_pending;
    doc["stable_ms"]  = (uint32_t)(millis() - s_stable_start_ms);
    doc["threshold_ms"] = (uint32_t)GUARD_STABLE_MS;
    doc["max_boots"]  = GUARD_MAX_BOOTS_BEFORE_ROLLBACK;
}
