// ============================================================
// Green Home P485 – ota_guard.h
// Bulletproof OTA: boot-counter alapu auto-rollback.
//
// Mukodes:
//  1) otaGuardBegin() a setup() legelejen: noveli a "bootok_ota_ota"
//     szamlalot az NVS-ben. Ha > GUARD_MAX_BOOTS_BEFORE_ROLLBACK,
//     automatikusan visszaallitja a masik OTA slotra a boot partit
//     es reboot. A korabbi mukodo image visszajon.
//  2) otaGuardLoop() main loop-bol: amikor az eszkoz stabil
//     (WiFi + web futott >= GUARD_STABLE_MS), egyszer meghivja
//     otaGuardMarkGood()-ot ami torli a boot-szamlalot.
//  3) otaGuardForceRollback() kezi visszallitas (HTTP endpoint).
//  4) otaGuardStatus() JSON statuszriport (/ota-status endpoint).
// ============================================================
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// Konfig
#ifndef GUARD_MAX_BOOTS_BEFORE_ROLLBACK
#  define GUARD_MAX_BOOTS_BEFORE_ROLLBACK 3
#endif
#ifndef GUARD_STABLE_MS
#  define GUARD_STABLE_MS 120000UL   // 2 perc stabil futas = jo image
#endif

void  otaGuardBegin();            // setup() eleje (WiFi elott!)
void  otaGuardLoop();             // main loop (mark-good ha stabil)
bool  otaGuardMarkGood();         // kezileg jonak jeloles
bool  otaGuardForceRollback();    // kezi rollback
bool  otaGuardIsPending();        // true = uj image meg nem lett jonak jelolve
void  otaGuardStatus(JsonDocument& doc);
uint32_t otaGuardBootCount();     // hany boot volt ota mark-good ota
const char* otaGuardPartitionLabel();
