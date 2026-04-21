// ============================================================
// Green Home P485 Modified – config.h
// Minden típusdefiníció, struct, konstans
// FIGYELEM: Az eredeti P485_backup_full_4MB.bin ÉRINTETLEN!
// ============================================================
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// ---- Firmware azonosítók ------------------------------------
#ifndef FW_VERSION
#  define FW_VERSION  "1.0.2"
#endif
#ifndef FW_PRODUCT
#  define FW_PRODUCT  "P485-GH"
#endif
#ifndef FW_BRAND
#  define FW_BRAND    "Green Home"
#endif

// ---- P485 MAC → topic alap + kliens ID ---------------------
#ifndef P485_MAC
#  define P485_MAC "84_1F_E8_15_B7_7C"
#endif

// ---- MQTT broker -------------------------------------------
// Pentasun támogatás eltávolítva. Csak egy GH/publikus broker marad.
#ifndef GH_MQTT_HOST
#  define GH_MQTT_HOST "test.mosquitto.org"
#endif
#ifndef GH_MQTT_PORT
#  define GH_MQTT_PORT 1883
#endif

// ---- Topic struktúra ----------------------------------------
// A MAC cím teszi egyedivé, publikus brokeren nehezebben kiszúrható
#define GH_DATA_TOPIC    "green_home/" P485_MAC "/data"
#define GH_HB_TOPIC      "green_home/" P485_MAC "/hb"
#define GH_STATUS_TOPIC  "green_home/" P485_MAC "/status"
#define GH_CMD_TOPIC     "green_home/" P485_MAC "/cmd"
// Green Home bridge heartbeat (visszirányú felügyelet)
#define GH_BRIDGE_HB_TOPIC "green_home/gh_bridge/hb"

// ---- Home Assistant MQTT Discovery -------------------------
#define GH_HA_DISC_PFX    "homeassistant"
#define GH_HA_DEV_ID      "p485_" P485_MAC
#define GH_HA_SET_PFX     "green_home/" P485_MAC "/ha/set/"
#define GH_HA_STATE_TOPIC "green_home/" P485_MAC "/ha/state"
#define GH_HA_SUB_TOPIC   "green_home/" P485_MAC "/ha/set/#"

// ---- Hardware pinout ----------------------------------------
#ifndef RS485_TX
#  define RS485_TX 17
#endif
#ifndef RS485_RX
#  define RS485_RX 16
#endif
#ifndef RS485_DE
#  define RS485_DE 4
#endif
#ifndef LED_STATUS
#  define LED_STATUS 2
#endif
#ifndef BUTTON_PIN
#  define BUTTON_PIN 0
#endif
#ifndef BUTTON_LONG_MS
#  define BUTTON_LONG_MS 5000
#endif

// ---- Rendszer konstansok ------------------------------------
#ifndef SERIAL_BAUD
#  define SERIAL_BAUD 115200
#endif
#ifndef WEB_PORT
#  define WEB_PORT 80
#endif
#ifndef MDNS_HOSTNAME
#  define MDNS_HOSTNAME "p485"
#endif
#ifndef MODBUS_SLAVE_ID
#  define MODBUS_SLAVE_ID 1
#endif
#ifndef MODBUS_BAUD
#  define MODBUS_BAUD 9600
#endif
#ifndef HB_INTERVAL_MS
#  define HB_INTERVAL_MS 10000
#endif
#ifndef LFS_CONFIG_PATH
#  define LFS_CONFIG_PATH "/config/settings.json"
#endif
#ifndef LFS_LOG_PATH
#  define LFS_LOG_PATH "/log/events.json"
#endif
#ifndef OTA_VERSION_URL
#  define OTA_VERSION_URL ""
#endif
#ifndef OTA_FIRMWARE_URL
#  define OTA_FIRMWARE_URL ""
#endif

// ============================================================
// DeyeData – Deye inverter mért értékek
// Regiszterek: Holding (FC03), slave ID konfigurálható
// Dokumentáció: Deye SUN series Modbus protocol v1.7
// ============================================================
struct DeyeData {
    // --- PV (napelem) – Deye SG04LP3, 2 MPPT ---
    float pv1_v      = 0;    // V   reg 676  × 0.1
    float pv1_a      = 0;    // A   reg 677  × 0.1
    float pv1_w      = 0;    // W   reg 672  (MPPT1 power, direct)
    float pv2_v      = 0;    // V   reg 678  × 0.1
    float pv2_a      = 0;    // A   reg 679  × 0.1
    float pv2_w      = 0;    // W   reg 673  (MPPT2 power, direct)
    float pv_total_w = 0;    // W   számított: pv1_w + pv2_w

    // --- Akkumulátor ---
    float bat_v      = 0;    // V   reg 587  × 0.01
    float bat_a      = 0;    // A   reg 591  × 0.01 signed (SG04LP3)
    float bat_w      = 0;    // W   reg 590  signed (+tölt, -kisüt)
    float bat_soc    = 0;    // %   reg 588  (0-100)
    float bat_temp   = 0;    // °C  reg 586  (raw-1000)×0.1

    // --- Hőmérsékletek ---
    float temp_dc    = 0;    // °C  reg 540  (raw-1000)×0.1  inverter DC oldal
    float temp_ac    = 0;    // °C  reg 541  (raw-1000)×0.1  hűtőborda/AC oldal

    // --- Energia számlálók (kWh, ×0.1) ---
    // Napi (éjfélkor nullázódik)
    float e_pv_day     = 0;  // kWh  reg 529  × 0.1
    float e_bat_chg    = 0;  // kWh  reg 514  × 0.1  napi akku töltés
    float e_bat_dis    = 0;  // kWh  reg 515  × 0.1  napi akku kisütés
    float e_grid_buy   = 0;  // kWh  reg 520  × 0.1  napi hálózat vásárlás
    float e_grid_sell  = 0;  // kWh  reg 521  × 0.1  napi hálózat eladás
    float e_load_day   = 0;  // kWh  reg 526  × 0.1  napi fogyasztás
    // Összesített
    float et_bat_chg   = 0;  // kWh  reg 516  × 0.1  összes akku töltés
    float et_bat_dis   = 0;  // kWh  reg 518  × 0.1  összes akku kisütés
    float et_grid_buy  = 0;  // kWh  reg 522  × 0.1  összes hálózat vásárlás
    float et_grid_sell = 0;  // kWh  reg 524  × 0.1  összes hálózat eladás
    float et_load      = 0;  // kWh  reg 527  × 0.1  összes fogyasztás

    // --- Hálózat – SG04LP3 regisztertérkép ---
    float grid_u_l1  = 0;   // V   reg 598  × 0.1
    float grid_u_l2  = 0;   // V   reg 599  × 0.1
    float grid_u_l3  = 0;   // V   reg 600  × 0.1
    float grid_i_l1  = 0;   // A   reg 630  × 0.01 signed (inverter AC kimenet)
    float grid_i_l2  = 0;   // A   reg 631  × 0.01 signed
    float grid_i_l3  = 0;   // A   reg 632  × 0.01 signed
    float grid_w_l1  = 0;   // W   reg 616  signed (external CT L1)
    float grid_w_l2  = 0;   // W   reg 617  signed (external CT L2)
    float grid_w_l3  = 0;   // W   reg 618  signed (external CT L3)
    float grid_w_total = 0; // W   reg 625  signed (+import, -export)
    float grid_freq  = 0;   // Hz  reg 609  × 0.01

    // --- Terhelés ---
    float load_w_total = 0; // W   számított: PV + grid - bat

    // --- Státusz ---
    uint16_t status  = 0;   // reg 512 (bit field)
    bool     valid   = false;
    uint32_t last_update_ms = 0;

    // --- Energiamérleg (számított) ---
    // Törvény: PV + grid_import = load + bat_charge + grid_export
    float balance_delta = 0;   // eltérés W-ban
    bool  balance_ok    = false;

    void calcBalance() {
        float production = pv_total_w;
        float import_w   = max(0.0f,  grid_w_total);
        float export_w   = max(0.0f, -grid_w_total);
        float bat_charge = max(0.0f,  bat_w);

        float in  = production + import_w;
        float out = load_w_total + bat_charge + export_w;
        balance_delta = fabsf(in - out);
        balance_ok    = (balance_delta < 300.0f);  // 300W tolerancia
    }

    // JSON kimenet – mindkét brokerre ezt küldjük
    String toJson() const {
        JsonDocument doc;
        doc["ts"]       = (unsigned long)(millis() / 1000);
        doc["pv1_w"]    = (int)pv1_w;
        doc["pv2_w"]    = (int)pv2_w;
        doc["pv_w"]     = (int)pv_total_w;
        doc["bat_v"]    = serialized(String(bat_v, 2));
        doc["bat_w"]    = (int)bat_w;
        doc["bat_soc"]  = (int)bat_soc;
        doc["bat_tmp"]  = serialized(String(bat_temp, 1));
        doc["temp_dc"]  = serialized(String(temp_dc, 1));
        doc["temp_ac"]  = serialized(String(temp_ac, 1));
        // Napi energia (kWh)
        doc["e_pv"]     = serialized(String(e_pv_day, 1));
        doc["e_bchg"]   = serialized(String(e_bat_chg, 1));
        doc["e_bdis"]   = serialized(String(e_bat_dis, 1));
        doc["e_buy"]    = serialized(String(e_grid_buy, 1));
        doc["e_sell"]   = serialized(String(e_grid_sell, 1));
        doc["e_load"]   = serialized(String(e_load_day, 1));
        // Összesített energia (kWh)
        doc["et_bchg"]  = serialized(String(et_bat_chg, 1));
        doc["et_bdis"]  = serialized(String(et_bat_dis, 1));
        doc["et_buy"]   = serialized(String(et_grid_buy, 1));
        doc["et_sell"]  = serialized(String(et_grid_sell, 1));
        doc["et_load"]  = serialized(String(et_load, 1));
        doc["grid_w"]   = (int)grid_w_total;
        doc["grid_u1"]  = serialized(String(grid_u_l1, 1));
        doc["grid_u2"]  = serialized(String(grid_u_l2, 1));
        doc["grid_u3"]  = serialized(String(grid_u_l3, 1));
        doc["grid_i1"]  = serialized(String(grid_i_l1, 2));
        doc["grid_i2"]  = serialized(String(grid_i_l2, 2));
        doc["grid_i3"]  = serialized(String(grid_i_l3, 2));
        doc["grid_ct1"] = (int)grid_w_l1;    // External CT L1 per-phase power (W, signed)
        doc["grid_ct2"] = (int)grid_w_l2;    // External CT L2 per-phase power (W, signed)
        doc["grid_ct3"] = (int)grid_w_l3;    // External CT L3 per-phase power (W, signed)
        doc["freq"]     = serialized(String(grid_freq, 2));
        doc["load_w"]   = (int)load_w_total;
        doc["bal_ok"]   = balance_ok;
        doc["bal_d"]    = (int)balance_delta;
        doc["stat"]     = status;
        doc["fw"]       = FW_VERSION;
        String out;
        serializeJson(doc, out);
        return out;
    }
};

// ============================================================
// P485Config – mentett beállítások (LittleFS)
// ============================================================
struct P485Config {
    char wifi_ssid[64]       = "";
    char wifi_pass[64]       = "";

    // Mi broker (Pentasun eltávolítva)
    char gh_host[128]        = GH_MQTT_HOST;
    uint16_t gh_port         = GH_MQTT_PORT;
    char gh_user[64]         = "";
    char gh_pass[64]         = "";

    // Modbus
    uint8_t  slave_id        = MODBUS_SLAVE_ID;
    uint32_t baud            = MODBUS_BAUD;

    // OTA URL-ek (MQTT CMD-ből is felülírható)
    char ota_url[256]        = "";
};

// ============================================================
// P485State – futási státusz
// ============================================================
struct P485State {
    bool     wifi_ok        = false;
    bool     gh_ok          = false;
    bool     modbus_ok      = false;
    bool     gh_peer_ok     = false;
    uint32_t poll_count     = 0;
    uint32_t error_count    = 0;
    uint16_t last_modbus_error = 0;
    uint16_t last_modbus_reg   = 0;
    float    last_poll_ms   = 0;
    uint32_t last_gh_hb_ms  = 0;
    char     local_ip[16]   = "0.0.0.0";
    uint32_t uptime_start   = 0;
};

// ---- Globális példányok (main.cpp definiálja) ---------------
extern P485Config  g_config;
extern P485State   g_state;
extern DeyeData    g_data;
