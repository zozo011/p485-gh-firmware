// ============================================================
// Green Home P485 – logger.h  (identikus a green_home-éval)
// LittleFS ring-buffer event log
// ============================================================
#pragma once
#include <Arduino.h>

class Logger {
public:
    static void begin();
    static void log(const char* event, const char* detail = "", const char* extra = "");
    static bool readEvents(String& out, uint8_t limit = 100);
    static void clear();

private:
    static const char* _path;
    static const int   _max_entries;
};
