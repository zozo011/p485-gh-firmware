// ============================================================
// Green Home P485 - mqtt_dual.cpp
// (Pentasun eltavolitva. Egy AsyncMqttClient a GH brokerhez,
//  exponencialis backoff, SOHA nem restartol.)
// ============================================================
#include "mqtt_dual.h"
#include "logger.h"
#include <WiFi.h>
#include <time.h>

static constexpr uint32_t GH_PEER_TIMEOUT_MS = 30000;

static MqttDual* _inst = nullptr;

void MqttDual::_gTimerCb(TimerHandle_t t) {
    if (_inst) _inst->_connectG();
}

void mqttDualReconnect() {
    if (_inst) _inst->reconnect();
}

void MqttDual::begin() {
    _inst = this;

    if (!_g_timer)
        _g_timer = xTimerCreate("gMqtt", pdMS_TO_TICKS(5000), pdFALSE, this, _gTimerCb);

    _gh.setClientId("gh_" P485_MAC);
    _gh.setServer(g_config.gh_host, g_config.gh_port);
    if (g_config.gh_user[0])
        _gh.setCredentials(g_config.gh_user, g_config.gh_pass);
    _gh.setKeepAlive(15);
    _gh.setWill(GH_HB_TOPIC, 0, false, "{\"alive\":false,\"lwt\":true}");

    _gh.onConnect([this](bool session) {
        _g_connected  = true;
        _g_delay      = 5000;
        g_state.gh_ok = true;
        Serial.println("[MQTT] GH OK");
        Logger::log("mqtt_gh", "connected");
        _gh.subscribe(GH_CMD_TOPIC, 0);
        _gh.subscribe(GH_BRIDGE_HB_TOPIC, 0);
        _gh.subscribe(GH_HA_SUB_TOPIC, 0);  // HA set parancsok
        publishHB();
        publishHADiscovery();   // HA entitasok automatikus regisztracio
    });

    _gh.onDisconnect([this](AsyncMqttClientDisconnectReason r) {
        _g_connected  = false;
        g_state.gh_ok = false;
        _g_delay = min(_g_delay * 2, (uint32_t)300000);
        Serial.printf("[MQTT] GH le, ujra %us\n", _g_delay / 1000);
        xTimerChangePeriod(_g_timer, pdMS_TO_TICKS(_g_delay), 0);
        xTimerStart(_g_timer, 0);
    });

    _gh.onMessage([this](char* topic, char* payload,
                          AsyncMqttClientMessageProperties props,
                          size_t len, size_t idx, size_t tot) {
        if (len == 0 || idx > 0) return;
        char buf[512];
        memcpy(buf, payload, min(len, sizeof(buf) - 1));
        buf[min(len, sizeof(buf) - 1)] = 0;
        if (strcmp(topic, GH_BRIDGE_HB_TOPIC) == 0) {
            _handleGhHeartbeat(buf);
            return;
        }
        if (strncmp(topic, GH_HA_SET_PFX, strlen(GH_HA_SET_PFX)) == 0) {
            _handleHASet(topic + strlen(GH_HA_SET_PFX), buf);
            return;
        }
        _handleCmd(topic, buf);
    });

    _connectG();
}

void MqttDual::_connectG() {
    if (WiFi.status() == WL_CONNECTED && !_gh.connected()) {
        Serial.println("[MQTT] Kapcsolodas a GH brokerhez...");
        _gh.connect();
    }
}

void MqttDual::reconnect() {
    _g_delay = 5000;
    if (_gh.connected()) _gh.disconnect();
    delay(100);
    _connectG();
}

void MqttDual::loop() {
    if (millis() - _last_hb > HB_INTERVAL_MS) {
        _last_hb = millis();
        publishHB();
    }

    if (g_state.last_gh_hb_ms > 0 && (millis() - g_state.last_gh_hb_ms) > GH_PEER_TIMEOUT_MS) {
        if (g_state.gh_peer_ok) {
            g_state.gh_peer_ok = false;
            Logger::log("peer_gh", "timeout");
            Serial.println("[PEER] Green Home heartbeat timeout");
        }
    }
}

void MqttDual::publishData(const DeyeData& data) {
    if (!_g_connected) return;

    String json = data.toJson();
    _gh.publish(GH_DATA_TOPIC, 0, false, json.c_str());

    digitalWrite(LED_STATUS, LOW);
    digitalWrite(LED_STATUS, HIGH);
}

void MqttDual::publishHB() {
    if (!_g_connected) return;

    JsonDocument doc;
    doc["alive"]     = true;
    doc["modbus_ok"] = g_state.modbus_ok;
    doc["uptime_s"]  = millis() / 1000;
    doc["heap"]      = esp_get_free_heap_size();
    doc["poll_ms"]   = (int)g_state.last_poll_ms;
    doc["polls"]     = g_state.poll_count;
    doc["errs"]      = g_state.error_count;
    doc["ip"]        = g_state.local_ip;
    doc["peer_gh"]   = g_state.gh_peer_ok;
    doc["fw"]        = FW_VERSION;
    // NTP-alapu pontos ora (Europe/Budapest) heartbeat-ben
    struct tm tinfo;
    if (getLocalTime(&tinfo, 10)) {
        char tbuf[24];
        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tinfo);
        doc["local_time"] = tbuf;
        doc["ntp_ok"]     = true;
    } else {
        doc["ntp_ok"]     = false;
    }

    String out;
    serializeJson(doc, out);
    _gh.publish(GH_HB_TOPIC, 0, false, out.c_str());
}

void MqttDual::_handleCmd(const char* topic, const char* payload) {
    Serial.printf("[MQTT CMD] topic=%s payload=%s\n", topic, payload);
    Logger::log("mqtt_cmd", topic, payload);

    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;

    const char* cmd = doc["cmd"] | "";

    if (strcmp(cmd, "restart") == 0) {
        Logger::log("restart", "mqtt_cmd");
        delay(200);
        esp_restart();

    } else if (strcmp(cmd, "ota") == 0) {
        const char* url = doc["url"] | "";
        if (url[0]) {
            // Ha URL-t kap (direkt bin vagy version.txt), hasznaljuk a WithUrl varianssal
            // hogy _override_url legyen beallitva es a version check ki legyen kerulve
            extern void otaTriggerWithUrl(const char* binUrl);
            otaTriggerWithUrl(url);
        }

    } else if (strcmp(cmd, "set_broker") == 0) {
        const char* host = doc["host"] | "";
        uint16_t    port = doc["port"] | 1883;
        if (host[0]) {
            strlcpy(g_config.gh_host, host, sizeof(g_config.gh_host));
            g_config.gh_port = port;
            reconnect();
        }

    } else if (strcmp(cmd, "write") == 0) {
        uint16_t reg = doc["reg"] | 0;
        uint16_t val = doc["val"] | 0;
        if (reg > 0) {
            extern bool safeWriteRegister(uint16_t reg, uint16_t value, String& errorMsg);
            String err;
            bool ok = safeWriteRegister(reg, val, err);

            JsonDocument resp;
            resp["cmd"]  = "write_result";
            resp["reg"]  = reg;
            resp["val"]  = val;
            resp["ok"]   = ok;
            if (!ok) resp["error"] = err;
            String out; serializeJson(resp, out);
            if (_g_connected)
                _gh.publish(GH_STATUS_TOPIC, 0, false, out.c_str());
        }

    } else if (strcmp(cmd, "get_settings") == 0) {
        extern bool readSettings(JsonDocument& doc);
        JsonDocument resp;
        resp["cmd"] = "settings";
        readSettings(resp);
        String out; serializeJson(resp, out);
        if (_g_connected)
            _gh.publish(GH_STATUS_TOPIC, 0, false, out.c_str());
        // HA state frissitese a beolvasott ertekekkel
        const char* HA_CTRL_KEYS[] = {
            "solar_sell_en","grid_charge_en","work_mode",
            "max_sell_power_w","max_bat_charge_a","max_bat_discharge_a","max_grid_charge_a"
        };
        for (const auto& k : HA_CTRL_KEYS) {
            if (!resp[k].isNull()) _ha_state[k] = resp[k];
        }
        publishHAState();

    } else {
        if (_cmd_cb) _cmd_cb(cmd, doc["val"] | "");
    }
}

void MqttDual::_handleGhHeartbeat(const char* payload) {
    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;

    bool alive = doc["alive"] | true;
    g_state.last_gh_hb_ms = millis();
    g_state.gh_peer_ok = alive;

    if (!alive) {
        Logger::log("peer_gh", "lwt_false");
        Serial.println("[PEER] Green Home reported alive:false");
    }
}

// ============================================================
// publishHAState – HA control allapot kuldese (retained)
// ============================================================
void MqttDual::publishHAState() {
    if (!_g_connected) return;
    if (_ha_state.size() == 0) return;
    String out; serializeJson(_ha_state, out);
    _gh.publish(GH_HA_STATE_TOPIC, 0, true, out.c_str());
}

// ============================================================
// _handleHASet – HA-bol erkező vezerlesi parancs feldolgozasa
// Pl. ha/set/solar_sell_en payload "1" vagy "0"
// ============================================================
void MqttDual::_handleHASet(const char* key, const char* payload) {
    struct RegMap { const char* key; uint16_t reg; };
    static const RegMap MAP[] = {
        {"solar_sell_en",      145},
        {"grid_charge_en",     130},
        {"work_mode",          142},
        {"max_sell_power_w",   143},
        {"max_bat_charge_a",   108},
        {"max_bat_discharge_a",109},
        {"max_grid_charge_a",  128},
    };

    uint16_t reg = 0;
    for (const auto& m : MAP) {
        if (strcmp(key, m.key) == 0) { reg = m.reg; break; }
    }
    if (reg == 0) {
        Serial.printf("[HA] Ismeretlen key: %s\n", key);
        return;
    }

    int val = atoi(payload);  // HA command_template mar numerikust kuld
    extern bool safeWriteRegister(uint16_t, uint16_t, String&);
    String err;
    bool ok = safeWriteRegister(reg, (uint16_t)val, err);

    Serial.printf("[HA] set %s=%d reg=%u %s\n", key, val, reg, ok ? "OK" : err.c_str());
    Logger::log("ha_set", ok ? "ok" : "fail");

    if (ok) {
        // Optimista state frissites – nem kell modbus ujraolvasas
        _ha_state[key] = val;
        publishHAState();
    }
}

// ============================================================
// publishHADiscovery – HA entitasok auto-regisztracio
// Minden retained QoS0 => HA azonnali felismerés indulas utan
// ============================================================
void MqttDual::publishHADiscovery() {
    if (!_g_connected) return;

    // --- Kozos device blokk lambda ---
    auto buildDev = [](JsonDocument& doc) {
        JsonArray ids = doc["device"]["identifiers"].to<JsonArray>();
        ids.add(GH_HA_DEV_ID);
        doc["device"]["name"]         = "P485 Green Home Dongle";
        doc["device"]["model"]        = "P485/ADA485 ESP32";
        doc["device"]["manufacturer"] = "Green Home Technologies";
        doc["device"]["sw_version"]   = FW_VERSION;
    };

    // --- SENSOR entitasok ---
    struct SensorDef {
        const char* key;
        const char* name;
        const char* unit;
        const char* dev_cls;
        const char* st_cls;
    };
    static const SensorDef SENSORS[] = {
        {"pv_w",    "PV Teljesítmény",           "W",   "power",       "measurement"},
        {"pv1_w",   "PV1 Teljesítmény",           "W",   "power",       "measurement"},
        {"pv2_w",   "PV2 Teljesítmény",           "W",   "power",       "measurement"},
        {"bat_soc", "Akku SOC",                   "%",   "battery",     "measurement"},
        {"bat_v",   "Akku Feszültség",             "V",   "voltage",     "measurement"},
        {"bat_w",   "Akku Teljesítmény",           "W",   "power",       "measurement"},
        {"bat_tmp", "Akku Hőmérséklet",            "°C",  "temperature", "measurement"},
        {"grid_w",  "Hálózat Teljesítmény",        "W",   "power",       "measurement"},
        {"grid_u1", "Hálózat Feszültség L1",       "V",   "voltage",     "measurement"},
        {"grid_u2", "Hálózat Feszültség L2",       "V",   "voltage",     "measurement"},
        {"grid_u3", "Hálózat Feszültség L3",       "V",   "voltage",     "measurement"},
        {"freq",    "Hálózati Frekvencia",          "Hz",  "frequency",   "measurement"},
        {"load_w",  "Fogyasztás",                   "W",   "power",       "measurement"},
        {"e_pv",    "Napi PV Termelés",             "kWh", "energy",      "total_increasing"},
        {"e_buy",   "Napi Vásárlás",                "kWh", "energy",      "total_increasing"},
        {"e_sell",  "Napi Eladás",                  "kWh", "energy",      "total_increasing"},
        {"e_bchg",  "Napi Akku Töltés",             "kWh", "energy",      "total_increasing"},
        {"e_bdis",  "Napi Akku Kisütés",            "kWh", "energy",      "total_increasing"},
        {"temp_dc", "Inverter DC Hőmérséklet",      "°C",  "temperature", "measurement"},
        {"temp_ac", "Inverter AC Hőmérséklet",      "°C",  "temperature", "measurement"},
        {"stat",    "Inverter Állapot",             nullptr, nullptr,     "measurement"},
    };

    for (const auto& s : SENSORS) {
        JsonDocument doc;
        buildDev(doc);
        doc["name"]              = s.name;
        doc["unique_id"]         = String(GH_HA_DEV_ID) + "_" + s.key;
        doc["state_topic"]       = GH_DATA_TOPIC;
        doc["value_template"]    = String("{{ value_json.") + s.key + " }}";
        if (s.unit)    doc["unit_of_measurement"] = s.unit;
        if (s.dev_cls) doc["device_class"]        = s.dev_cls;
        if (s.st_cls)  doc["state_class"]         = s.st_cls;
        doc["availability_topic"]        = GH_HB_TOPIC;
        doc["availability_template"]     = "{{ 'online' if value_json.alive else 'offline' }}";
        String topic = String(GH_HA_DISC_PFX) + "/sensor/" GH_HA_DEV_ID "/" + s.key + "/config";
        String out; serializeJson(doc, out);
        _gh.publish(topic.c_str(), 0, true, out.c_str());
        delay(25);
    }

    // --- SWITCH entitasok ---
    struct SwDef { const char* key; const char* name; };
    static const SwDef SWITCHES[] = {
        {"solar_sell_en",  "Napelem Értékesítés"},
        {"grid_charge_en", "Hálózati Töltés"},
    };
    for (const auto& sw : SWITCHES) {
        JsonDocument doc;
        buildDev(doc);
        doc["name"]                  = sw.name;
        doc["unique_id"]             = String(GH_HA_DEV_ID) + "_" + sw.key;
        doc["state_topic"]           = GH_HA_STATE_TOPIC;
        doc["value_template"]        = String("{{ value_json.") + sw.key + " }}";
        doc["state_on"]              = "1";
        doc["state_off"]             = "0";
        doc["command_topic"]         = String(GH_HA_SET_PFX) + sw.key;
        doc["payload_on"]            = "1";
        doc["payload_off"]           = "0";
        doc["availability_topic"]    = GH_HB_TOPIC;
        doc["availability_template"] = "{{ 'online' if value_json.alive else 'offline' }}";
        String topic = String(GH_HA_DISC_PFX) + "/switch/" GH_HA_DEV_ID "/" + sw.key + "/config";
        String out; serializeJson(doc, out);
        _gh.publish(topic.c_str(), 0, true, out.c_str());
        delay(25);
    }

    // --- NUMBER entitasok ---
    struct NumDef { const char* key; const char* name; const char* unit; int mn; int mx; int step; };
    static const NumDef NUMBERS[] = {
        {"max_sell_power_w",    "Max Eladási Teljesítmény", "W",  0, 8000, 100},
        {"max_bat_charge_a",    "Max Akku Töltő Áram",      "A",  0,  200,   5},
        {"max_bat_discharge_a", "Max Akku Kisütő Áram",     "A",  0,  200,   5},
        {"max_grid_charge_a",   "Max Hálózati Töltő Áram",  "A",  0,  200,   5},
    };
    for (const auto& n : NUMBERS) {
        JsonDocument doc;
        buildDev(doc);
        doc["name"]                  = n.name;
        doc["unique_id"]             = String(GH_HA_DEV_ID) + "_" + n.key;
        doc["state_topic"]           = GH_HA_STATE_TOPIC;
        doc["value_template"]        = String("{{ value_json.") + n.key + " }}";
        doc["command_topic"]         = String(GH_HA_SET_PFX) + n.key;
        doc["unit_of_measurement"]   = n.unit;
        doc["min"]                   = n.mn;
        doc["max"]                   = n.mx;
        doc["step"]                  = n.step;
        doc["mode"]                  = "box";
        doc["availability_topic"]    = GH_HB_TOPIC;
        doc["availability_template"] = "{{ 'online' if value_json.alive else 'offline' }}";
        String topic = String(GH_HA_DISC_PFX) + "/number/" GH_HA_DEV_ID "/" + n.key + "/config";
        String out; serializeJson(doc, out);
        _gh.publish(topic.c_str(), 0, true, out.c_str());
        delay(25);
    }

    // --- SELECT: work_mode ---
    {
        JsonDocument doc;
        buildDev(doc);
        doc["name"]                  = "Üzemmód";
        doc["unique_id"]             = String(GH_HA_DEV_ID) + "_work_mode";
        doc["state_topic"]           = GH_HA_STATE_TOPIC;
        doc["value_template"]        = "{% set m = value_json.work_mode | int %}"
                                       "{% if m == 0 %}Sell"
                                       "{% elif m == 2 %}Zero Export"
                                       "{% else %}Unknown{% endif %}";
        doc["command_topic"]         = String(GH_HA_SET_PFX) + "work_mode";
        doc["command_template"]      = "{% if value == 'Sell' %}0"
                                       "{% elif value == 'Zero Export' %}2"
                                       "{% else %}0{% endif %}";
        JsonArray opts = doc["options"].to<JsonArray>();
        opts.add("Sell");
        opts.add("Zero Export");
        doc["availability_topic"]    = GH_HB_TOPIC;
        doc["availability_template"] = "{{ 'online' if value_json.alive else 'offline' }}";
        String topic = String(GH_HA_DISC_PFX) + "/select/" GH_HA_DEV_ID "/work_mode/config";
        String out; serializeJson(doc, out);
        _gh.publish(topic.c_str(), 0, true, out.c_str());
    }

    Serial.println("[HA] Discovery published (" GH_HA_DEV_ID ")");
    Logger::log("ha_disc", "ok");
}
