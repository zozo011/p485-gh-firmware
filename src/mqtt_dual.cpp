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
        publishHB();
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
            strlcpy(g_config.ota_url, url, sizeof(g_config.ota_url));
            extern void otaTriggerCheck();
            otaTriggerCheck();
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
