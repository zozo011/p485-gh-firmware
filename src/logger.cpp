// ============================================================
// Green Home P485 – logger.cpp  (identikus a green_home-éval)
// LittleFS ring-buffer event log, legfeljebb 100 bejegyzés
// ============================================================
#include "logger.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <time.h>

const char* Logger::_path        = LFS_LOG_PATH;
const int   Logger::_max_entries = 100;

void Logger::begin() {
    if (!LittleFS.exists("/log"))
        LittleFS.mkdir("/log");
    if (!LittleFS.exists(_path)) {
        File f = LittleFS.open(_path, "w");
        if (f) { f.print("[]"); f.close(); }
    }
}

void Logger::log(const char* event, const char* detail, const char* extra) {
    File f = LittleFS.open(_path, "r");
    JsonDocument doc;
    if (f) {
        DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err) { doc.clear(); doc.to<JsonArray>(); }
    } else {
        doc.to<JsonArray>();
    }

    JsonArray arr = doc.as<JsonArray>();

    JsonDocument entry;
    char ts[32] = "N/A";
    struct tm t;
    time_t now = time(nullptr);
    if (now > 1700000000 && localtime_r(&now, &t)) {
        snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        snprintf(ts, sizeof(ts), "+%lus", (unsigned long)(millis() / 1000));
    }

    entry["t"] = ts;
    entry["e"] = event;
    if (detail && detail[0]) entry["d"] = detail;
    if (extra  && extra[0])  entry["x"] = extra;
    entry["u"] = (unsigned long)millis();
    arr.add(entry);

    Serial.printf("[LOG] %s | %s | %s\n", ts, event, detail);

    while ((int)arr.size() > _max_entries)
        arr.remove(0);

    File fw = LittleFS.open(_path, "w");
    if (fw) { serializeJson(doc, fw); fw.close(); }
}

bool Logger::readEvents(String& out, uint8_t limit) {
    File f = LittleFS.open(_path, "r");
    if (!f) { out = "[]"; return false; }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) { out = "[]"; return false; }

    JsonArray arr = doc.as<JsonArray>();
    size_t sz  = arr.size();
    size_t start = (sz > limit) ? sz - limit : 0;

    JsonDocument slice;
    JsonArray    sa = slice.to<JsonArray>();
    for (size_t i = start; i < sz; i++)
        sa.add(arr[i]);

    serializeJson(slice, out);
    return true;
}

void Logger::clear() {
    File f = LittleFS.open(_path, "w");
    if (f) { f.print("[]"); f.close(); }
}
