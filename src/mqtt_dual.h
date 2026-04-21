// ============================================================
// Green Home P485 - mqtt_dual.h
// (Pentasun eltavolitva. Csak a GH broker marad,
//  a filenevek valtozatlanok.)
// ============================================================
#pragma once
#include <Arduino.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "config.h"

class MqttDual {
public:
    void begin();
    void loop();

    void publishData(const DeyeData& data);
    void publishHB();
    void publishHADiscovery();
    void publishHAState();

    bool ghOk() const { return _g_connected; }

    using CmdCallback = std::function<void(const char* cmd, const char* val)>;
    void onCommand(CmdCallback cb) { _cmd_cb = cb; }

    void reconnect();

private:
    AsyncMqttClient _gh;

    TimerHandle_t _g_timer = nullptr;

    uint32_t _g_delay    = 5000;
    bool     _g_connected = false;

    uint32_t _last_hb    = 0;

    CmdCallback _cmd_cb;

    void _connectG();
    void _handleCmd(const char* topic, const char* payload);
    void _handleHASet(const char* key, const char* payload);
    void _handleGhHeartbeat(const char* payload);

    JsonDocument _ha_state;  // utolso ismert HA control allapot (retained)

    static void _gTimerCb(TimerHandle_t t);
};

extern MqttDual  g_mqtt;
void mqttDualReconnect();
