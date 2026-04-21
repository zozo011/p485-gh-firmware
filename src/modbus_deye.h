// ============================================================
// Green Home P485 – modbus_deye.h
// Deye SUN sorozat inverter regiszter lekérdezés (batch read)
// FC03 Holding Registers, slave ID konfigurálható
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"

class ModbusDeye {
public:
    enum ErrorCode : uint16_t {
        ERR_NONE            = 0,
        ERR_INVALID_PARAM   = 100,
        ERR_TIMEOUT_NODATA  = 201,
        ERR_SHORT_FRAME     = 202,
        ERR_CRC             = 203,
        ERR_FRAME_HEADER    = 204,
        ERR_BYTECOUNT       = 205
    };

    void begin();

    // Egy gyors próbakérés a status regiszterre (512), automata baud/slave kereséshez.
    bool probe(uint8_t slave, uint32_t baud);

    // Teljes Deye lekérdezés – batch módban, amint kész: true
    // Visszatér a tényleges poll idővel (ms)
    bool poll(DeyeData& data);

    // Nyers regiszter olvasás (debug/scan célra)
    // Visszatér: olvasott szavak száma, -1 ha hiba
    int readRaw(uint16_t start, uint16_t count, uint16_t* out);

    // Egyetlen regiszter írása (FC06 Write Single Register)
    // Visszatér: true ha echo OK
    bool writeRegister(uint16_t reg, uint16_t value);

    uint32_t errorCount()   const { return _errors; }
    uint32_t successCount() const { return _ok; }
    uint16_t lastError()    const { return _last_error; }
    uint16_t lastRegister() const { return _last_reg; }

private:
    bool _initialized = false;
    uint32_t _current_baud = 0;
    uint32_t _errors  = 0;
    uint32_t _ok      = 0;
    uint16_t _last_error = ERR_NONE;
    uint16_t _last_reg   = 0;

    // Batch read: több egymás utáni regiszter egy kérésben
    // data_out: big-endian szó-párok
    // Visszatér: valóban olvasott szavak száma, -1 hiba esetén
    int batchRead(uint8_t slave, uint8_t fc, uint16_t start,
                  uint16_t count, uint16_t* out);

    // Segédek
    uint16_t crc16(const uint8_t* buf, size_t len);
    float    toSigned(uint16_t raw)  { return (float)(int16_t)raw; }
    float    toUnsigned(uint16_t raw){ return (float)raw; }
};

extern ModbusDeye g_modbus;
