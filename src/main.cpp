// ============================================================
// Green Home P485 – main.cpp
// P485 ESP32 módosított firmware
// – Deye inverter Modbus olvasás (UART2, 9600, FC03)
// – Single MQTT: GH broker (test.mosquitto.org default)
// – OTA (24h + MQTT CMD)
// – Minimál web API: /sysinfo /log /config /restart
// – WiFiManager AP mód (első induláskor hálózatlista + jelaszó bevítel)
// ============================================================
#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <ArduinoOTA.h>
#include <Update.h>
#include <time.h>

#include "config.h"
#include "mqtt_dual.h"
#include "modbus_deye.h"
#include "logger.h"
#include "ota.h"
#include "ota_guard.h"

// Token a nem-whitelistelt ("unsafe") regiszter-irashoz es firmware-uploadhoz
#ifndef P485_ADMIN_TOKEN
#  define P485_ADMIN_TOKEN "green2026"
#endif

// ============================================================
// GLOBALSOK
// ============================================================
P485Config  g_config;
P485State   g_state;
DeyeData    g_data;

MqttDual    g_mqtt;
ModbusDeye  g_modbus;

static AsyncWebServer webServer(WEB_PORT);
static SemaphoreHandle_t xDataMutex = nullptr;

static void configSave();

static uint32_t s_last_auto_detect_ms = 0;
static uint32_t s_last_log_error_code = 0xFFFFFFFF;
static uint32_t s_last_log_error_reg  = 0xFFFFFFFF;
static volatile bool s_poll_paused = false;
static volatile uint32_t s_poll_pause_until = 0;

// ============================================================
// INVERTER WRITE – biztonságos regiszter írás
// Engedélyezett regiszterek whitelist (biztonsági okokból!)
// ============================================================
struct AllowedReg {
    uint16_t reg;
    uint16_t min_val;
    uint16_t max_val;
    const char* name;
};

static const AllowedReg s_allowed_regs[] = {
    // Battery fizikai limitek
    { 103,   0, 1000, "bat_capacity_ah"     },   // Battery capacity (Ah)
    { 104, 3000, 6000, "bat_shutdown_v_x100"},   // Shutdown V (×0.01, 30-60V)
    { 105, 3000, 6000, "bat_restart_v_x100" },   // Restart V
    { 106, 3000, 6000, "bat_low_alarm_v_x100"},  // Low alarm V
    { 107, 3000, 6000, "bat_resume_v_x100"  },   // Resume V
    { 108,   0,  200, "max_bat_charge_a"    },   // Max battery charge current (A)
    { 109,   0,  200, "max_bat_discharge_a" },   // Max battery discharge current (A)
    // Charge voltages
    { 112, 4000, 6000, "float_charge_v_x100"},   // Float V
    { 113, 4000, 6000, "equal_charge_v_x100"},   // Equalization V
    { 114, 4000, 6000, "abs_charge_v_x100"  },   // Absorption V
    // Grid charge + peak shaving
    { 127,   0, 6000, "grid_shaving_v_x100" },   // Peak shaving battery V
    { 128,   0,  200, "max_grid_charge_a"   },   // Max grid charge A
    { 129,   0,    1, "grid_peak_shave_en"  },   // Peak shaving enable
    { 130,   0,    1, "grid_charge_en"      },   // Grid charge enable
    { 131,   0,    1, "gen_charge_en"       },   // Generator charge enable
    { 132,   0,    1, "gen_signal_en"       },   // Generator signal enable
    // Work mode
    { 141,   0,  100, "bat_empty_soc_pct"   },   // Low SOC cutoff (%)
    { 142,   0,    2, "work_mode"           },   // 0=SellFirst, 1=ZeroExport(load), 2=ZeroExport(CT)
    { 143,   0,12000, "max_sell_power_w"    },   // Max solar sell power (W)
    { 144,   0,    1, "grid_export_limit_en"},   // Grid export limit en
    { 145,   0,    1, "solar_sell_en"       },   // Solar sell enable
    { 146,   0,12000, "grid_peak_shave_w"   },   // Peak shaving power (W)
    // TOU – Time of Use
    { 148,   0,    1, "tou_en"              },   // TOU enable (0/1)
    { 149,   0,    1, "tou_ac_charge_en"    },   // AC charge en (TOU window)
    // TOU slotok (6 idoszak) — 166-171 kezdoido (HH*100+MM), 172-177 min SOC
    { 166,   0, 2359, "tou_t1_start"        },
    { 167,   0, 2359, "tou_t2_start"        },
    { 168,   0, 2359, "tou_t3_start"        },
    { 169,   0, 2359, "tou_t4_start"        },
    { 170,   0, 2359, "tou_t5_start"        },
    { 171,   0, 2359, "tou_t6_start"        },
    // TOU min SOC per slot
    { 172,   0,  100, "tou_t1_soc_pct"      },
    { 173,   0,  100, "tou_t2_soc_pct"      },
    { 174,   0,  100, "tou_t3_soc_pct"      },
    { 175,   0,  100, "tou_t4_soc_pct"      },
    { 176,   0,  100, "tou_t5_soc_pct"      },
    { 177,   0,  100, "tou_t6_soc_pct"      },
    // Rendszer ora (NTP-sync)
    {  22, 2000, 2099, "clock_year"         },
    {  23,    1,   12, "clock_month"        },
    {  24,    1,   31, "clock_day"          },
    {  25,    0,   23, "clock_hour"         },
    {  26,    0,   59, "clock_minute"       },
    {  27,    0,   59, "clock_second"       },
    // Legacy
    { 340,   0,12000, "legacy_max_sell_w"   },
};
static constexpr int s_allowed_count = sizeof(s_allowed_regs) / sizeof(s_allowed_regs[0]);

struct ObservedReg {
    uint16_t reg;
    const char* name;
    const char* note;
};

static const ObservedReg s_observed_regs[] = {
    { 108, "max_bat_charge_a",    "current_fw" },
    { 109, "max_bat_discharge_a", "current_fw" },
    { 128, "max_grid_charge_a",   "current_fw" },
    { 130, "grid_charge_en",      "current_fw" },
    { 142, "work_mode",           "current_fw" },
    { 143, "max_sell_power_w",    "current_fw" },
    { 145, "solar_sell_en",       "current_fw" },
    { 340, "legacy_max_sell_power_w", "historical_doc_candidate" },
};
static constexpr int s_observed_count = sizeof(s_observed_regs) / sizeof(s_observed_regs[0]);

static const AllowedReg* findAllowedReg(uint16_t reg) {
    for (int i = 0; i < s_allowed_count; i++) {
        if (s_allowed_regs[i].reg == reg) return &s_allowed_regs[i];
    }
    return nullptr;
}

bool readControlRegisters(JsonDocument& doc) {
    s_poll_pause_until = millis() + 3000;
    s_poll_paused = true;
    delay(200);

    JsonArray regs = doc["registers"].to<JsonArray>();

    for (int i = 0; i < s_observed_count; i++) {
        const ObservedReg& info = s_observed_regs[i];
        JsonObject r = regs.add<JsonObject>();
        r["reg"] = info.reg;
        r["name"] = info.name;
        r["note"] = info.note;

        const AllowedReg* allowed = findAllowedReg(info.reg);
        r["writable"] = (allowed != nullptr);
        if (allowed) {
            r["min"] = allowed->min_val;
            r["max"] = allowed->max_val;
        }

        uint16_t raw = 0;
        int res = g_modbus.readRaw(info.reg, 1, &raw);
        if (res == 1) {
            r["ok"] = true;
            r["u"] = raw;
            r["s"] = (int16_t)raw;
        } else {
            r["ok"] = false;
            r["error"] = g_modbus.lastError();
        }

        delay(50);
    }

    return true;
}

// Regiszter írás (poll-pause + whitelist + range check)
bool safeWriteRegister(uint16_t reg, uint16_t value, String& errorMsg) {
    const AllowedReg* ar = findAllowedReg(reg);
    if (!ar) {
        errorMsg = "register not allowed";
        return false;
    }
    if (value < ar->min_val || value > ar->max_val) {
        errorMsg = String("value out of range [") + ar->min_val + "-" + ar->max_val + "]";
        return false;
    }

    // Poll szüneteltetés – 2.5s várakozás hogy a poll ciklus biztosan befejezze
    s_poll_pause_until = millis() + 5000;
    s_poll_paused = true;
    delay(2500);

    bool ok = g_modbus.writeRegister(reg, value);
    if (!ok) {
        errorMsg = String("modbus write failed (err=") + g_modbus.lastError() + ")";
    } else {
        char detail[64];
        snprintf(detail, sizeof(detail), "reg%u=%u(%s)", reg, value, ar->name);
        Logger::log("inverter_write", detail);
    }
    return ok;
}

// Settings olvasás (több regiszter egyszerre, poll-pause-szal)
bool readSettings(JsonDocument& doc) {
    // Jelezzük a poll tasknak hogy szuneteljen, majd varunk 1.5 ciklust (1.5s)
    // hogy biztosan befejezze a folyamatban levo Modbus kommunikaciot
    s_poll_pause_until = millis() + 8000;
    s_poll_paused = true;
    delay(1500);

    uint16_t buf[16];
    int res;

    // ---- Inverter ora (reg 22-27) ----
    res = g_modbus.readRaw(22, 6, buf);
    if (res == 6) {
        char tbuf[32];
        snprintf(tbuf, sizeof(tbuf), "%04u-%02u-%02u %02u:%02u:%02u",
                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        doc["inverter_time"] = tbuf;
    }
    delay(50);

    // ---- Battery params (reg 103-114, 12 szo) ----
    res = g_modbus.readRaw(103, 12, buf);
    if (res == 12) {
        doc["bat_capacity_ah"]     = buf[0];              // 103
        doc["bat_shutdown_v"]      = buf[1] * 0.01f;      // 104
        doc["bat_restart_v"]       = buf[2] * 0.01f;      // 105
        doc["bat_low_alarm_v"]     = buf[3] * 0.01f;      // 106
        doc["bat_resume_v"]        = buf[4] * 0.01f;      // 107
        doc["max_bat_charge_a"]    = buf[5];              // 108
        doc["max_bat_discharge_a"] = buf[6];              // 109
        // 110-111 reserved
        doc["float_charge_v"]      = buf[9]  * 0.01f;     // 112
        doc["equal_charge_v"]      = buf[10] * 0.01f;     // 113
        doc["abs_charge_v"]        = buf[11] * 0.01f;     // 114
    }
    delay(50);

    // ---- Grid charge + gen (reg 127-132, 6 szo) ----
    res = g_modbus.readRaw(127, 6, buf);
    if (res == 6) {
        doc["grid_shaving_v"]    = buf[0] * 0.01f;   // 127
        doc["max_grid_charge_a"] = buf[1];           // 128
        doc["grid_peak_shave_en"]= buf[2];           // 129
        doc["grid_charge_en"]    = buf[3];           // 130
        doc["gen_charge_en"]     = buf[4];           // 131
        doc["gen_signal_en"]     = buf[5];           // 132
    }
    delay(50);

    // ---- Work mode reg 141-146 (6 szo) ----
    res = g_modbus.readRaw(141, 6, buf);
    if (res == 6) {
        doc["bat_empty_soc_pct"]   = buf[0];  // 141
        doc["work_mode"]           = buf[1];  // 142
        doc["max_sell_power_w"]    = buf[2];  // 143
        doc["grid_export_limit_en"]= buf[3];  // 144
        doc["solar_sell_en"]       = buf[4];  // 145
        doc["grid_peak_shave_w"]   = buf[5];  // 146
    }
    delay(50);

    // ---- TOU enable + slots (reg 148-149, 166-177) ----
    res = g_modbus.readRaw(148, 2, buf);
    if (res == 2) {
        doc["tou_en"]           = buf[0];
        doc["tou_ac_charge_en"] = buf[1];
    }
    delay(50);

    res = g_modbus.readRaw(166, 12, buf);
    if (res == 12) {
        JsonArray tou = doc["tou_slots"].to<JsonArray>();
        for (int i = 0; i < 6; i++) {
            JsonObject slot = tou.add<JsonObject>();
            slot["start"]    = buf[i];       // HH*100+MM
            slot["min_soc"]  = buf[i + 6];   // %
        }
    }
    delay(50);

    return true;
}

static bool tryAutoDetectModbus() {
    struct Candidate {
        uint32_t baud;
        uint8_t slave;
    };

    static const Candidate kCandidates[] = {
        {9600, 1},
        {9600, 2},
        {19200, 1},
        {19200, 2},
        {4800, 1},
    };

    for (const auto& c : kCandidates) {
        if (g_modbus.probe(c.slave, c.baud)) {
            g_config.slave_id = c.slave;
            g_config.baud = c.baud;
            configSave();
            Logger::log("modbus", "autodetect_ok");
            Serial.printf("[MODBUS] Auto-detect OK: slave=%u baud=%u\n", c.slave, c.baud);
            return true;
        }
    }

    return false;
}

// ============================================================
// KONFIG BETÖLTÉS / MENTÉS (LittleFS)
// ============================================================
static void configLoad() {
    File f = LittleFS.open(LFS_CONFIG_PATH, "r");
    if (!f) return;
    JsonDocument doc;
    if (deserializeJson(doc, f)) { f.close(); return; }
    f.close();

    { const char* v = doc["gh_host"] | ""; strlcpy(g_config.gh_host, (v[0] ? v : GH_MQTT_HOST), sizeof(g_config.gh_host)); }
    { uint16_t v = doc["gh_port"] | 0; g_config.gh_port = v ? v : GH_MQTT_PORT; }
    strlcpy(g_config.gh_user, doc["gh_user"] | "", sizeof(g_config.gh_user));
    strlcpy(g_config.gh_pass, doc["gh_pass"] | "", sizeof(g_config.gh_pass));

    g_config.slave_id = doc["slave_id"] | (uint8_t)MODBUS_SLAVE_ID;
    g_config.baud     = doc["baud"]     | (uint32_t)MODBUS_BAUD;

    strlcpy(g_config.ota_url, doc["ota_url"] | "", sizeof(g_config.ota_url));
    Serial.println("[CFG] Konfig betöltve.");
}

static void configSave() {
    if (!LittleFS.exists("/config"))
        LittleFS.mkdir("/config");
    File f = LittleFS.open(LFS_CONFIG_PATH, "w");
    if (!f) return;
    JsonDocument doc;
    doc["gh_host"]       = g_config.gh_host;
    doc["gh_port"]       = g_config.gh_port;
    doc["gh_user"]       = g_config.gh_user;
    doc["gh_pass"]       = g_config.gh_pass;
    doc["slave_id"]      = g_config.slave_id;
    doc["baud"]          = g_config.baud;
    doc["ota_url"]       = g_config.ota_url;
    serializeJson(doc, f);
    f.close();
    Serial.println("[CFG] Konfig mentve.");
}

// ============================================================
// WEB API
// ============================================================
static void setupWebServer() {
    // GET /sysinfo
    webServer.on("/sysinfo", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["fw"]          = FW_VERSION;
        doc["product"]     = FW_PRODUCT;
        doc["uptime_s"]    = millis() / 1000;
        doc["free_heap"]   = esp_get_free_heap_size();
        doc["ip"]          = WiFi.localIP().toString();
        doc["modbus_ok"]   = g_state.modbus_ok;
        doc["gh_ok"]       = g_state.gh_ok;
        doc["gh_peer_ok"]  = g_state.gh_peer_ok;
        doc["poll_count"]  = g_state.poll_count;
        doc["error_count"] = g_state.error_count;
        doc["last_modbus_error"] = g_state.last_modbus_error;
        doc["last_modbus_reg"]   = g_state.last_modbus_reg;
        doc["poll_ms"]     = (int)g_state.last_poll_ms;
        // NTP-alapu pontos ora (Europe/Budapest)
        struct tm tinfo;
        if (getLocalTime(&tinfo, 10)) {
            char tbuf[32];
            strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tinfo);
            doc["local_time"] = tbuf;
            doc["ntp_ok"]     = true;
        } else {
            doc["local_time"] = "";
            doc["ntp_ok"]     = false;
        }
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // GET /data – utolsó Deye mérés
    webServer.on("/data", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(100))) {
            String j = g_data.toJson();
            xSemaphoreGive(xDataMutex);
            req->send(200, "application/json", j);
        } else {
            req->send(503, "application/json", "{\"error\":\"busy\"}");
        }
    });

    // GET /log
    webServer.on("/log", HTTP_GET, [](AsyncWebServerRequest* req) {
        String out;
        Logger::readEvents(out, 100);
        req->send(200, "application/json", out);
    });

    // POST /log/clear
    webServer.on("/log/clear", HTTP_POST, [](AsyncWebServerRequest* req) {
        Logger::clear();
        req->send(200, "application/json", "{\"ok\":true}");
    });

    // GET /config
    webServer.on("/config", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["gh_host"]       = g_config.gh_host;
        doc["gh_port"]       = g_config.gh_port;
        doc["gh_user"]       = g_config.gh_user;
        // gh_pass szándékosan kihagyva
        doc["slave_id"]      = g_config.slave_id;
        doc["baud"]          = g_config.baud;
        doc["ota_url"]       = g_config.ota_url;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // POST /config (JSON body)
    webServer.on("/config", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t idx, size_t tot) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) {
                req->send(400, "application/json", "{\"error\":\"invalid json\"}");
                return;
            }
            if (doc["gh_host"].is<const char*>())      strlcpy(g_config.gh_host, doc["gh_host"], sizeof(g_config.gh_host));
            if (doc["gh_port"].is<uint16_t>())         g_config.gh_port = doc["gh_port"];
            if (doc["gh_user"].is<const char*>())      strlcpy(g_config.gh_user, doc["gh_user"], sizeof(g_config.gh_user));
            if (doc["gh_pass"].is<const char*>())      strlcpy(g_config.gh_pass, doc["gh_pass"], sizeof(g_config.gh_pass));
            if (doc["slave_id"].is<uint8_t>())         g_config.slave_id = doc["slave_id"];
            if (doc["baud"].is<uint32_t>())            g_config.baud     = doc["baud"];
            if (doc["ota_url"].is<const char*>())      strlcpy(g_config.ota_url, doc["ota_url"], sizeof(g_config.ota_url));
            configSave();
            req->send(200, "application/json", "{\"ok\":true,\"restart\":true}");
        }
    );

    // POST /restart
    webServer.on("/restart", HTTP_POST, [](AsyncWebServerRequest* req) {
        req->send(200, "application/json", "{\"ok\":true}");
        Logger::log("restart", "web");
        delay(300);
        esp_restart();
    });

    // POST /ota
    webServer.on("/ota", HTTP_POST, [](AsyncWebServerRequest* req) {
        otaTriggerCheck();
        req->send(200, "application/json", "{\"ok\":true,\"msg\":\"OTA check triggered\"}");
    });

    // GET /regscan?start=512&count=10 – nyers regiszter olvasás (debug)
    webServer.on("/regscan", HTTP_GET, [](AsyncWebServerRequest* req) {
        uint16_t start = 512;
        uint16_t count = 10;
        if (req->hasParam("start")) start = req->getParam("start")->value().toInt();
        if (req->hasParam("count")) count = req->getParam("count")->value().toInt();
        if (count > 50) count = 50;  // biztonsági limit

        // Poll task szüneteltetés (3 mp)
        s_poll_pause_until = millis() + 3000;
        s_poll_paused = true;
        delay(200);  // várjunk, hogy a poll task befejezze az aktuális olvasást

        uint16_t buf[50];
        int res = g_modbus.readRaw(start, count, buf);

        JsonDocument doc;
        doc["start"] = start;
        doc["count"] = count;

        if (res > 0) {
            doc["ok"] = true;
            doc["read"] = res;
            JsonArray raw = doc["raw"].to<JsonArray>();
            JsonArray regs = doc["regs"].to<JsonArray>();
            for (int i = 0; i < res; i++) {
                raw.add(buf[i]);
                // Olvasható formátum: "reg=value (signed)"
                JsonObject r = regs.add<JsonObject>();
                r["r"] = start + i;
                r["u"] = buf[i];
                r["s"] = (int16_t)buf[i];
                r["f01"] = buf[i] * 0.01f;
                r["f1"] = buf[i] * 0.1f;
            }
        } else {
            doc["ok"] = false;
            doc["error"] = g_modbus.lastError();
        }

        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // GET /settings – inverter beállítások lekérdezése
    webServer.on("/settings", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["ok"] = readSettings(doc);

        // Engedélyezett regiszterek listája (UI-nak)
        JsonArray allowed = doc["allowed"].to<JsonArray>();
        for (int i = 0; i < s_allowed_count; i++) {
            JsonObject a = allowed.add<JsonObject>();
            a["reg"]  = s_allowed_regs[i].reg;
            a["name"] = s_allowed_regs[i].name;
            a["min"]  = s_allowed_regs[i].min_val;
            a["max"]  = s_allowed_regs[i].max_val;
        }

        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // GET /control-registers – vezérlő regiszterek és történeti jelöltek kiolvasása
    webServer.on("/control-registers", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["ok"] = readControlRegisters(doc);
        doc["count"] = s_observed_count;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // POST /write – inverter regiszter írás (JSON body: {"reg":130,"val":1})
    webServer.on("/write", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t idx, size_t tot) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) {
                req->send(400, "application/json", "{\"error\":\"invalid json\"}");
                return;
            }

            if (!doc["reg"].is<uint16_t>() || !doc["val"].is<uint16_t>()) {
                req->send(400, "application/json", "{\"error\":\"missing reg/val\"}");
                return;
            }

            uint16_t reg = doc["reg"];
            uint16_t val = doc["val"];

            String errorMsg;
            bool ok = safeWriteRegister(reg, val, errorMsg);

            JsonDocument resp;
            resp["ok"]  = ok;
            resp["reg"] = reg;
            resp["val"] = val;
            if (!ok) resp["error"] = errorMsg;

            // Visszaolvasás ellenőrzésre
            if (ok) {
                delay(100);
                uint16_t readback;
                int r = g_modbus.readRaw(reg, 1, &readback);
                if (r == 1) {
                    resp["readback"] = readback;
                    resp["verified"] = (readback == val);
                }
            }

            String out; serializeJson(resp, out);
            req->send(ok ? 200 : 400, "application/json", out);
        }
    );

    // ---------------------------------------------------------
    // OTA guard statusz + manualis vezerles
    // ---------------------------------------------------------
    webServer.on("/ota-status", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        otaGuardStatus(doc);
        doc["fw"]      = FW_VERSION;
        doc["product"] = FW_PRODUCT;
        doc["uptime_s"]= millis() / 1000;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    webServer.on("/ota-mark-good", HTTP_POST, [](AsyncWebServerRequest* req) {
        bool ok = otaGuardMarkGood();
        req->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    webServer.on("/ota-rollback", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("token") || req->getParam("token")->value() != P485_ADMIN_TOKEN) {
            req->send(401, "application/json", "{\"error\":\"bad token\"}");
            return;
        }
        req->send(200, "application/json", "{\"ok\":true,\"msg\":\"rolling back\"}");
        Logger::log("ota_guard", "http_rollback");
        delay(300);
        otaGuardForceRollback();
    });

    // ---------------------------------------------------------
    // BROWSER FIRMWARE UPLOAD – bulletproof vesz-haboru recovery
    // POST /firmware-upload?token=... (multipart: "firmware" mezo)
    // Ha a felteto siker: eszkoz restart, boot-count inicializalodik
    // ---------------------------------------------------------
    webServer.on("/firmware-upload", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            bool ok = !Update.hasError();
            JsonDocument d;
            d["ok"]    = ok;
            d["size"]  = Update.size();
            d["progress"] = Update.progress();
            if (!ok) d["error"] = Update.errorString();
            String out; serializeJson(d, out);
            AsyncWebServerResponse* resp = req->beginResponse(ok ? 200 : 500, "application/json", out);
            resp->addHeader("Connection", "close");
            req->send(resp);
            if (ok) {
                Logger::log("firmware_upload", "ok");
                delay(500);
                esp_restart();
            }
        },
        [](AsyncWebServerRequest* req, String filename, size_t index, uint8_t* data, size_t len, bool final_) {
            if (index == 0) {
                // Token check a legelso chunk-on
                if (!req->hasParam("token") || req->getParam("token")->value() != P485_ADMIN_TOKEN) {
                    Serial.println("[UPLOAD] Tiltott: rossz token");
                    return;
                }
                Serial.printf("[UPLOAD] Start: %s\n", filename.c_str());
                Logger::log("firmware_upload", "start", filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                }
            }
            if (!Update.hasError()) {
                if (Update.write(data, len) != len) {
                    Update.printError(Serial);
                }
            }
            if (final_) {
                if (Update.end(true)) {
                    Serial.printf("[UPLOAD] Kesz: %u byte\n", (unsigned)(index + len));
                } else {
                    Update.printError(Serial);
                    Logger::log("firmware_upload", "end_fail");
                }
            }
        }
    );

    // Minimalis bongeszo-UI a felteteshez
    webServer.on("/firmware", HTTP_GET, [](AsyncWebServerRequest* req) {
        const char* html =
            "<!doctype html><html><body style='font-family:sans-serif;max-width:480px;margin:20px auto'>"
            "<h2>P485 firmware upload</h2>"
            "<p>FW: " FW_VERSION "</p>"
            "<form method='POST' action='/firmware-upload?token=" P485_ADMIN_TOKEN "' enctype='multipart/form-data'>"
            "<input type='file' name='firmware' accept='.bin' required><br><br>"
            "<button type='submit'>Upload &amp; restart</button>"
            "</form>"
            "<p><a href='/ota-status'>OTA statusz</a> | <a href='/sysinfo'>sysinfo</a></p>"
            "</body></html>";
        req->send(200, "text/html", html);
    });

    // ---------------------------------------------------------
    // RAW (UNSAFE) regiszter iras – token-gated, BARMELY regiszterre
    // POST /regwrite-raw {"reg":N,"val":V,"token":"..."}
    // Veszelyes! Csak akkor hasznalja a user, ha tudja mit csinal.
    // ---------------------------------------------------------
    webServer.on("/regwrite-raw", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t idx, size_t tot) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) {
                req->send(400, "application/json", "{\"error\":\"invalid json\"}"); return;
            }
            const char* token = doc["token"] | "";
            if (strcmp(token, P485_ADMIN_TOKEN) != 0) {
                req->send(401, "application/json", "{\"error\":\"bad token\"}"); return;
            }
            if (!doc["reg"].is<uint16_t>() || !doc["val"].is<uint16_t>()) {
                req->send(400, "application/json", "{\"error\":\"missing reg/val\"}"); return;
            }
            uint16_t reg = doc["reg"], val = doc["val"];

            s_poll_pause_until = millis() + 5000;
            s_poll_paused = true;
            delay(2500);

            bool ok = g_modbus.writeRegister(reg, val);
            char detail[64]; snprintf(detail, sizeof(detail), "raw_reg%u=%u", reg, val);
            Logger::log("inverter_write", detail);

            JsonDocument resp;
            resp["ok"]  = ok;
            resp["reg"] = reg;
            resp["val"] = val;
            resp["raw"] = true;
            if (ok) {
                delay(100);
                uint16_t rb;
                if (g_modbus.readRaw(reg, 1, &rb) == 1) {
                    resp["readback"] = rb;
                    resp["verified"] = (rb == val);
                }
            } else {
                resp["error"] = g_modbus.lastError();
            }
            String out; serializeJson(resp, out);
            req->send(ok ? 200 : 500, "application/json", out);
        }
    );

    // GET /regread?start=N&count=M – szelesebb diagnosztikai olvasas (max 100)
    webServer.on("/regread", HTTP_GET, [](AsyncWebServerRequest* req) {
        uint16_t start = 0, count = 1;
        if (req->hasParam("start")) start = req->getParam("start")->value().toInt();
        if (req->hasParam("count")) count = req->getParam("count")->value().toInt();
        if (count == 0) count = 1;
        if (count > 60) count = 60;

        s_poll_pause_until = millis() + 3000;
        s_poll_paused = true;
        delay(200);

        uint16_t buf[64];
        int res = g_modbus.readRaw(start, count, buf);

        JsonDocument doc;
        doc["start"] = start; doc["count"] = count;
        if (res > 0) {
            doc["ok"] = true;
            JsonArray arr = doc["values"].to<JsonArray>();
            for (int i = 0; i < res; i++) arr.add(buf[i]);
        } else {
            doc["ok"] = false;
            doc["error"] = g_modbus.lastError();
        }
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    webServer.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "application/json", "{\"error\":\"not found\"}");
    });

    webServer.begin();
    Serial.println("[WEB] API szerver indítva.");
}

// ============================================================
// MODBUS POLL TASK – Core 0, amint kész → azonnal küld
// ============================================================
static void modbusPollTask(void* pvParam) {
    g_modbus.begin();
    Serial.println("[MODBUS] Poll task indult (Core 0).");

    while (true) {
        // Poll pause (regscan endpoint kérésére)
        if (s_poll_paused && millis() < s_poll_pause_until) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        s_poll_paused = false;

        DeyeData tmp;
        uint32_t t0 = millis();
        bool ok = g_modbus.poll(tmp);
        uint32_t dt = millis() - t0;

        g_state.last_poll_ms = (float)dt;
        g_state.last_modbus_error = g_modbus.lastError();
        g_state.last_modbus_reg   = g_modbus.lastRegister();

        if (ok) {
            g_state.modbus_ok  = true;
            g_state.poll_count++;

            // Adatok thread-safe másolása
            if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(50))) {
                g_data = tmp;
                xSemaphoreGive(xDataMutex);
            }

            // Azonnal küld (async, nem blokkoló)
            g_mqtt.publishData(tmp);

        } else {
            g_state.modbus_ok = false;
            g_state.error_count++;
            Serial.printf("[MODBUS] Poll hiba #%u\n", g_state.error_count);
            char detail[48];
            snprintf(detail, sizeof(detail), "poll_error_e%u_r%u",
                     (unsigned)g_state.last_modbus_error,
                     (unsigned)g_state.last_modbus_reg);

            const bool errorChanged =
                (s_last_log_error_code != g_state.last_modbus_error) ||
                (s_last_log_error_reg  != g_state.last_modbus_reg);
            if (errorChanged || (g_state.error_count % 20 == 1)) {
                Logger::log("modbus", detail);
                s_last_log_error_code = g_state.last_modbus_error;
                s_last_log_error_reg  = g_state.last_modbus_reg;
            }

            // Ha nincs még egyetlen sikeres olvasás sem, időnként próbáljunk auto-detectet.
            uint32_t now = millis();
            if (g_state.poll_count == 0 && (now - s_last_auto_detect_ms) > 15000) {
                s_last_auto_detect_ms = now;
                if (tryAutoDetectModbus()) {
                    Logger::log("modbus", "autodetect_reconfig");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        // Nincs fix delay! Amint a batch read kész, indul a következő.
        // A batch read saját 50ms inter-request delay-t tartalmaz.
        taskYIELD();
    }
}

// ============================================================
// LED BLINK TASK – állapot jelzés
// ============================================================
static void ledTask(void* pvParam) {
    pinMode(LED_STATUS, OUTPUT);
    while (true) {
        if (!g_state.wifi_ok) {
            // WiFi nincs: gyors villogás
            digitalWrite(LED_STATUS, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(LED_STATUS, LOW);  vTaskDelay(pdMS_TO_TICKS(100));
        } else if (!g_state.modbus_ok) {
            // WiFi OK, Modbus hiba: kettős villogás
            for (int i = 0; i < 2; i++) {
                digitalWrite(LED_STATUS, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
                digitalWrite(LED_STATUS, LOW);  vTaskDelay(pdMS_TO_TICKS(100));
            }
            vTaskDelay(pdMS_TO_TICKS(600));
        } else {
            // Minden OK: lassú villogás
            digitalWrite(LED_STATUS, HIGH); vTaskDelay(pdMS_TO_TICKS(900));
            digitalWrite(LED_STATUS, LOW);  vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.printf("\n\n=== %s %s v%s ===\n", FW_BRAND, FW_PRODUCT, FW_VERSION);
    Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());

    // OTA rollback guard – LEHETO LEGKORABBAN, meg a WiFi elott!
    // Ha tul sokszor bootoltunk mark-good nelkul, rollback + restart.
    otaGuardBegin();

    // LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] LittleFS hiba! Formázás...");
        LittleFS.format();
        LittleFS.begin(true);
    }
    Logger::begin();
    Logger::log("boot", FW_VERSION);

    // Konfig betöltés
    configLoad();

    // Mutex
    xDataMutex = xSemaphoreCreateMutex();

    // WiFiManager – első induláskor AP mód: hálózatlista + jelaszó bevítel
    // Később az elmentett cred-del automatikusan csatlakozik
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);  // 3 perc, aztán restart
    wm.setConnectTimeout(20);

    // Extra paraméterek a portal alján (opcionális, előre kitöltve)
    WiFiManagerParameter p_gh("gh_host",       "GH MQTT host",     g_config.gh_host, 127);
    WiFiManagerParameter p_gp("gh_port",       "GH MQTT port",     String(g_config.gh_port).c_str(), 6);
    wm.addParameter(&p_gh); wm.addParameter(&p_gp);

    bool connected = wm.autoConnect("P485_Setup", "12345678");
    if (!connected) {
        Logger::log("wifi", "portal_timeout");
        esp_restart();
    }

    // Portal értékek visszaolvasása (csak ha út lett állítva)
    strlcpy(g_config.gh_host, p_gh.getValue(), sizeof(g_config.gh_host));
    g_config.gh_port = atoi(p_gp.getValue());
    configSave();

    // IP mentés
    strlcpy(g_state.local_ip, WiFi.localIP().toString().c_str(), sizeof(g_state.local_ip));
    g_state.wifi_ok = true;
    Serial.printf("[WiFi] IP: %s\n", g_state.local_ip);
    Logger::log("wifi", "connected", g_state.local_ip);

    // ---- NTP sync (CET+DST, magyarorszagi ido) ----
    configTime(0, 0, "pool.ntp.org", "time.cloudflare.com", "time.google.com");
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);  // Europe/Budapest
    tzset();
    Serial.println("[NTP] Sync indul (pool.ntp.org)...");

    // mDNS
    if (MDNS.begin(MDNS_HOSTNAME))
        Serial.printf("[mDNS] http://%s.local\n", MDNS_HOSTNAME);

    // Web API – leghamarabb indul, hogy mindig elérhető legyen
    setupWebServer();
    Serial.println("[WEB] API elérhető.");

    // ArduinoOTA – WiFi-n keresztül firmware frissítés
    ArduinoOTA.setHostname(MDNS_HOSTNAME);
    ArduinoOTA.setPassword("green2026");
    ArduinoOTA.onStart([]() { Serial.println("[OTA-A] Start..."); });
    ArduinoOTA.onEnd([]()   { Serial.println("[OTA-A] Kész, restart..."); });
    ArduinoOTA.onError([](ota_error_t err) {
        Serial.printf("[OTA-A] Hiba: %u\n", err);
    });
    ArduinoOTA.begin();
    Serial.println("[OTA-A] ArduinoOTA kész.");

    // MQTT dual
    g_mqtt.begin();

    // HTTP OTA
    otaSetup();

    // Tasks
    xTaskCreatePinnedToCore(modbusPollTask, "modbus", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(ledTask,        "led",    1024, nullptr, 1, nullptr, 1);

    Serial.println("[MAIN] Indítás kész.");
    Logger::log("startup", "ok", g_state.local_ip);
}

// ============================================================
// LOOP – core 1 (AsyncWebServer + MQTT loop + OTA)
// ============================================================
void loop() {
    ArduinoOTA.handle();
    g_mqtt.loop();  // heartbeat watchdog
    otaLoop();
    otaGuardLoop(); // 120 mp stabil futas utan mark-good
    delay(10);
}
