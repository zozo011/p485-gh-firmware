// ============================================================
// Green Home P485 – ota.h
// HTTP OTA + MQTT CMD trigger
// ============================================================
#pragma once
#include <Arduino.h>

void otaSetup();
void otaLoop();
void otaTriggerCheck();                     // web_server / MQTT CMD hívja
void otaTriggerWithUrl(const char* binUrl); // MQTT "ota" CMD -> URL megadva
