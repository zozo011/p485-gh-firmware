// ============================================================
// Green Home P485 – modbus_deye.cpp
// Deye SUN inverter Modbus RTU – batch regiszter lekérdezés
//
// Regiszter térkép (FC03, slave ID=1):
//   512      : run status
//   524-528  : bat_v, bat_a, bat_w, bat_soc, bat_temp
//   533-543  : grid U/I/P L1-L3 + total + freq
//   546-551  : PV1 + PV2 (V, A, W)
//   575      : PV total W
//   588-591  : load L1-L3 + total
//
// Batch olvasás: max 5 kérés/ciklus → ~800-1200ms
// ============================================================
#include "modbus_deye.h"
#include "logger.h"
#include <HardwareSerial.h>

int ModbusDeye::readRaw(uint16_t start, uint16_t count, uint16_t* out) {
    if (!_initialized) return -1;
    return batchRead(g_config.slave_id, 3, start, count, out);
}

static HardwareSerial rs485(2);

// DE pin vezérlés
static inline void deEn(bool tx) {
    digitalWrite(RS485_DE, tx ? HIGH : LOW);
}

// =============================================================
// CRC16 (Modbus standard)
// =============================================================
uint16_t ModbusDeye::crc16(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

// =============================================================
// BEGIN
// =============================================================
void ModbusDeye::begin() {
    pinMode(RS485_DE, OUTPUT);
    deEn(false);
    rs485.begin(g_config.baud, SERIAL_8N1, RS485_RX, RS485_TX);
    _current_baud = g_config.baud;
    Serial.printf("[Deye] UART2: baud=%d TX=%d RX=%d DE=%d slave=%d\n",
                  g_config.baud, RS485_TX, RS485_RX, RS485_DE, g_config.slave_id);
    _initialized = true;
}

bool ModbusDeye::probe(uint8_t slave, uint32_t baud) {
    if (!_initialized) {
        return false;
    }

    if (_current_baud != baud) {
        rs485.end();
        rs485.begin(baud, SERIAL_8N1, RS485_RX, RS485_TX);
        _current_baud = baud;
    }

    uint16_t word = 0;
    int res = batchRead(slave, 3, 512, 1, &word);
    return (res == 1);
}

// =============================================================
// WRITE REGISTER – FC16 (0x10) Write Multiple Registers
// Deye inverterek FC16-ot várnak, FC06-ot nem támogatják!
// Egy regiszter írása: 11 bájt kérés, 8 bájt válasz
// =============================================================
bool ModbusDeye::writeRegister(uint16_t reg, uint16_t value) {
    if (!_initialized) return false;

    uint8_t slave = g_config.slave_id;

    // FC16 frame: slave(1) + fc(1) + reg(2) + count(2) + bytecount(1) + data(2) = 9 bájt + CRC(2) = 11
    uint8_t req[11];
    req[0] = slave;
    req[1] = 0x10;              // FC16 Write Multiple Registers
    req[2] = (reg >> 8) & 0xFF;
    req[3] =  reg       & 0xFF;
    req[4] = 0x00;              // Register count high
    req[5] = 0x01;              // Register count low (1 register)
    req[6] = 0x02;              // Byte count (2 bytes per register)
    req[7] = (value >> 8) & 0xFF;
    req[8] =  value       & 0xFF;
    uint16_t c = crc16(req, 9);
    req[9]  = c & 0xFF;
    req[10] = (c >> 8) & 0xFF;

    // Flush RX
    while (rs485.available()) rs485.read();

    // TX
    deEn(true);
    rs485.write(req, 11);
    rs485.flush();
    deEn(false);

    // FC16 válasz: slave(1) + fc(1) + reg(2) + count(2) + CRC(2) = 8 bájt
    uint8_t resp[8];
    uint8_t rxlen = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 500 && rxlen < 8) {
        if (rs485.available())
            resp[rxlen++] = rs485.read();
        else
            delayMicroseconds(200);
    }

    if (rxlen < 8) {
        Serial.printf("[Deye] WRITE FC16 reg=%u val=%u FAIL: rxlen=%u\n", reg, value, rxlen);
        _last_error = ERR_SHORT_FRAME;
        return false;
    }

    // Válasz ellenőrzés: slave + fc=0x10 + reg + count=1
    bool ok = (resp[0] == slave && resp[1] == 0x10 &&
               resp[2] == req[2] && resp[3] == req[3] &&
               resp[4] == 0x00  && resp[5] == 0x01);

    // CRC ellenőrzés
    if (ok) {
        uint16_t resp_crc = resp[6] | (resp[7] << 8);
        uint16_t calc_crc = crc16(resp, 6);
        ok = (resp_crc == calc_crc);
    }

    if (ok) {
        Serial.printf("[Deye] WRITE FC16 reg=%u val=%u OK\n", reg, value);
        _last_error = ERR_NONE;
    } else {
        Serial.printf("[Deye] WRITE FC16 reg=%u val=%u FAIL: resp=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                      reg, value, resp[0], resp[1], resp[2], resp[3], resp[4], resp[5], resp[6], resp[7]);
        _last_error = ERR_FRAME_HEADER;
    }
    return ok;
}

// =============================================================
// BATCH READ – több regiszter egy kérésben (FC03)
// Visszatér: count ha rendben, -1 ha hiba
// =============================================================
int ModbusDeye::batchRead(uint8_t slave, uint8_t fc,
                           uint16_t start, uint16_t count,
                           uint16_t* out)
{
    _last_reg = start;
    if (count == 0 || count > 64) {
        _last_error = ERR_INVALID_PARAM;
        return -1;
    }

    // Kérés frame (8 bájt)
    uint8_t req[8];
    req[0] = slave;
    req[1] = fc;
    req[2] = (start >> 8) & 0xFF;
    req[3] =  start        & 0xFF;
    req[4] = (count >> 8) & 0xFF;
    req[5] =  count        & 0xFF;
    uint16_t c = crc16(req, 6);
    req[6] = c & 0xFF;
    req[7] = (c >> 8) & 0xFF;

    // Rx buffer flush
    while (rs485.available()) rs485.read();

    // TX
    deEn(true);
    rs485.write(req, 8);
    rs485.flush();
    deEn(false);

    // Várt válasz méret: slave(1)+fc(1)+bytecount(1)+data(2*count)+crc(2)
    uint8_t expected = 5 + count * 2;
    uint8_t rxbuf[150];
    uint8_t rxlen = 0;
    uint32_t t0 = millis();

    while (millis() - t0 < 300 && rxlen < expected) {
        if (rs485.available())
            rxbuf[rxlen++] = rs485.read();
        else
            delayMicroseconds(200);
    }

    // Minimum ellenőrzés
    if (rxlen == 0) {
        _last_error = ERR_TIMEOUT_NODATA;
        return -1;
    }
    if (rxlen < 5) {
        _last_error = ERR_SHORT_FRAME;
        return -1;
    }

    // CRC ellenőrzés
    uint16_t recv_crc = (uint16_t)rxbuf[rxlen-1] << 8 | rxbuf[rxlen-2];
    uint16_t calc_crc = crc16(rxbuf, rxlen - 2);
    if (recv_crc != calc_crc) {
        Serial.printf("[Deye] CRC hiba! reg=%d count=%d rxlen=%d\n", start, count, rxlen);
        _last_error = ERR_CRC;
        return -1;
    }

    // Frame ellenőrzés
    if (rxbuf[0] != slave || rxbuf[1] != fc) {
        _last_error = ERR_FRAME_HEADER;
        return -1;
    }
    uint8_t byte_cnt = rxbuf[2];
    if (byte_cnt != count * 2) {
        _last_error = ERR_BYTECOUNT;
        return -1;
    }

    // Szavak kibontása (big-endian)
    for (uint16_t i = 0; i < count; i++)
        out[i] = (uint16_t)rxbuf[3 + i*2] << 8 | rxbuf[4 + i*2];

    _last_error = ERR_NONE;
    return (int)count;
}

// =============================================================
// POLL – minden adat lekérdezése, visszatér ha kész
//
// Regiszter térkép – Deye SG04LP3 (verified by regscan + official map)
// Forrás: github.com/kbialek/deye-inverter-mqtt
//
//   512        : run status
//   514-529    : energy counters (daily + total kWh)
//   540-541    : temp DC/AC ((raw-1000)×0.1°C)
//   586        : bat_temp ((raw-1000)×0.1°C)
//   587        : bat_v (×0.01V)
//   588        : bat_soc (0-100%)
//   590        : bat_power (W, signed: +charge, -discharge)
//   591        : bat_current (×0.01A, signed)
//   598-600    : grid voltage L1/L2/L3 (×0.1V)
//   604-606    : internal CT L1/L2/L3 power (W, signed)
//   607        : total internal power (W, signed)
//   609        : freq (×0.01Hz)
//   616-618    : external CT L1/L2/L3 power (W, signed = grid per-phase)
//   625        : total grid power (W, signed = +import, -export)
//   630-632    : inverter output current L1/L2/L3 (×0.01A, signed)
//   672-673    : PV1/PV2 MPPT power (W, direct)
//   676-679    : PV1/PV2 voltage/current (×0.1)
// =============================================================
bool ModbusDeye::poll(DeyeData& data) {
    if (!_initialized) return false;

    uint8_t  slave = g_config.slave_id;
    uint16_t buf[64];
    int      res;

    data.valid = false;

    // ---- 1. Status (reg 512, 1 szó) — kritikus ----
    res = batchRead(slave, 3, 512, 1, buf);
    if (res == 1) {
        data.status = buf[0];
    } else {
        _errors++;
        Serial.printf("[Deye] reg512 FAIL: err=%u\n", _last_error);
        return false;
    }
    delay(50);

    // ---- 2. Battery base (reg 586-589, 4 szó) ----
    // 586=bat_temp, 587=bat_v, 588=bat_soc, 589=unused on SG04LP3
    res = batchRead(slave, 3, 586, 4, buf);
    if (res >= 3) {
        data.bat_temp  = (buf[0] - 1000) * 0.1f; // reg 586
        data.bat_v     = buf[1] * 0.01f;          // reg 587
        data.bat_soc   = buf[2] * 1.0f;           // reg 588
    } else {
        data.bat_v = data.bat_soc = data.bat_temp = 0;
        Serial.printf("[Deye] reg586-589 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 2b. Battery power + current (reg 590-591, 2 szó) ----
    res = batchRead(slave, 3, 590, 2, buf);
    if (res == 2) {
        data.bat_w = toSigned(buf[0]) * 1.0f;       // reg 590 (W, signed)
        data.bat_a = toSigned(buf[1]) * 0.01f;       // reg 591 (A, ×0.01 signed)
        Serial.printf("[Deye] Bat: %.1fV SOC=%.0f%% %.2fA %+.0fW tmp=%.1fC\n",
                      data.bat_v, data.bat_soc, data.bat_a, data.bat_w, data.bat_temp);
    } else {
        data.bat_w = data.bat_a = 0;
        Serial.printf("[Deye] reg590-591 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 3. Inverter hőmérsékletek (reg 540-541, 2 szó) ----
    res = batchRead(slave, 3, 540, 2, buf);
    if (res == 2) {
        data.temp_dc = (buf[0] - 1000) * 0.1f;  // reg 540 DC/belső
        data.temp_ac = (buf[1] - 1000) * 0.1f;  // reg 541 AC/hűtőborda
        Serial.printf("[Deye] Temp DC=%.1fC AC=%.1fC\n", data.temp_dc, data.temp_ac);
    } else {
        data.temp_dc = data.temp_ac = 0;
        Serial.printf("[Deye] reg540-541 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 4. Energia számlálók (reg 514-529, 16 szó) ----
    res = batchRead(slave, 3, 514, 16, buf);
    if (res == 16) {
        data.e_bat_chg   = buf[0] * 0.1f;   // reg 514 napi akku töltés
        data.e_bat_dis   = buf[1] * 0.1f;   // reg 515 napi akku kisütés
        data.et_bat_chg  = buf[2] * 0.1f;   // reg 516 összes akku töltés
        // buf[3] = reg 517 (high word, 0)
        data.et_bat_dis  = buf[4] * 0.1f;   // reg 518 összes akku kisütés
        // buf[5] = reg 519 (high word, 0)
        data.e_grid_buy  = buf[6] * 0.1f;   // reg 520 napi grid vásárlás
        data.e_grid_sell = buf[7] * 0.1f;   // reg 521 napi grid eladás
        data.et_grid_buy = buf[8] * 0.1f;   // reg 522 összes grid vásárlás
        // buf[9] = reg 523 (high word, 0)
        data.et_grid_sell= buf[10] * 0.1f;  // reg 524 összes grid eladás
        // buf[11] = reg 525 (high word, 0)
        data.e_load_day  = buf[12] * 0.1f;  // reg 526 napi fogyasztás
        data.et_load     = buf[13] * 0.1f;  // reg 527 összes fogyasztás
        // buf[14] = reg 528 (high word, 0)
        data.e_pv_day    = buf[15] * 0.1f;  // reg 529 napi PV termelés
        Serial.printf("[Deye] Energy day: PV=%.1f bat+%.1f/-%.1f buy=%.1f sell=%.1f load=%.1f kWh\n",
                      data.e_pv_day, data.e_bat_chg, data.e_bat_dis,
                      data.e_grid_buy, data.e_grid_sell, data.e_load_day);
    } else {
        Serial.printf("[Deye] reg514-529 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 5. Grid voltage + internal CT + freq (reg 598-609, 12 szó) ----
    res = batchRead(slave, 3, 598, 12, buf);
    if (res == 12) {
        data.grid_u_l1    = buf[0] * 0.1f;    // reg 598
        data.grid_u_l2    = buf[1] * 0.1f;    // reg 599
        data.grid_u_l3    = buf[2] * 0.1f;    // reg 600
        // buf[3..5] = reg 601-603 (reserved)
        // buf[6..8] = reg 604-606 = Internal CT L1-L3 Power (W, signed) — nem használjuk
        data.grid_freq    = buf[11] * 0.01f;  // reg 609

        Serial.printf("[Deye] Grid V: %.1f/%.1f/%.1f  freq=%.2fHz\n",
                      data.grid_u_l1, data.grid_u_l2, data.grid_u_l3, data.grid_freq);
    } else {
        Serial.printf("[Deye] reg598-609 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 5b. External CT per-phase (reg 616-618, 3 szó) = VALÓDI grid power/fázis ----
    res = batchRead(slave, 3, 616, 3, buf);
    if (res == 3) {
        data.grid_w_l1 = toSigned(buf[0]) * 1.0f;  // reg 616
        data.grid_w_l2 = toSigned(buf[1]) * 1.0f;  // reg 617
        data.grid_w_l3 = toSigned(buf[2]) * 1.0f;  // reg 618
        Serial.printf("[Deye] ExtCT W/phase: %+.0f/%+.0f/%+.0f\n",
                      data.grid_w_l1, data.grid_w_l2, data.grid_w_l3);
    } else {
        data.grid_w_l1 = data.grid_w_l2 = data.grid_w_l3 = 0;
        Serial.printf("[Deye] reg616-618 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 5c. Total Grid Power (reg 625, 1 szó) = VALÓDI hálózati teljesítmény ----
    res = batchRead(slave, 3, 625, 1, buf);
    if (res == 1) {
        data.grid_w_total = toSigned(buf[0]) * 1.0f;  // reg 625 (+import, -export)
        Serial.printf("[Deye] Grid total: %+.0fW (%s)\n", data.grid_w_total,
                      data.grid_w_total > 0 ? "IMPORT" : data.grid_w_total < 0 ? "EXPORT" : "BALANCED");
    } else {
        // Fallback: external CT összeg
        data.grid_w_total = data.grid_w_l1 + data.grid_w_l2 + data.grid_w_l3;
        Serial.printf("[Deye] reg625 FAIL: err=%u, fallback extCT=%+.0fW\n",
                      _last_error, data.grid_w_total);
    }
    delay(50);

    // ---- 6. Inverter output current per-phase (reg 630-632, 3 szó) ----
    res = batchRead(slave, 3, 630, 3, buf);
    if (res == 3) {
        data.grid_i_l1 = toSigned(buf[0]) * 0.01f;  // reg 630
        data.grid_i_l2 = toSigned(buf[1]) * 0.01f;  // reg 631
        data.grid_i_l3 = toSigned(buf[2]) * 0.01f;  // reg 632
    } else {
        data.grid_i_l1 = data.grid_i_l2 = data.grid_i_l3 = 0;
        Serial.printf("[Deye] reg630-632 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 7. PV MPPT power (reg 672-673, 2 szó) ----
    res = batchRead(slave, 3, 672, 2, buf);
    if (res == 2) {
        data.pv1_w = buf[0] * 1.0f;   // reg 672 MPPT1 W
        data.pv2_w = buf[1] * 1.0f;   // reg 673 MPPT2 W
        data.pv_total_w = data.pv1_w + data.pv2_w;  // számított összeg
        Serial.printf("[Deye] PV: MPPT1=%dW MPPT2=%dW total=%dW\n",
                      (int)data.pv1_w, (int)data.pv2_w, (int)data.pv_total_w);
    } else {
        data.pv1_w = data.pv2_w = data.pv_total_w = 0;
        Serial.printf("[Deye] reg672-673 FAIL: err=%u (skip)\n", _last_error);
    }
    delay(50);

    // ---- 7b. PV V/A details (reg 676-679, 4 szó) — diagnosztika ----
    res = batchRead(slave, 3, 676, 4, buf);
    if (res == 4) {
        data.pv1_v = buf[0] * 0.1f;   // reg 676
        data.pv1_a = buf[1] * 0.1f;   // reg 677
        data.pv2_v = buf[2] * 0.1f;   // reg 678
        data.pv2_a = buf[3] * 0.1f;   // reg 679
    } else {
        data.pv1_v = data.pv1_a = data.pv2_v = data.pv2_a = 0;
    }

    // ---- Validálás + LOAD SZÁMÍTÁS + mérleg ----
    data.last_update_ms = millis();
    data.valid = (data.grid_freq > 40.0f && data.grid_freq < 60.0f)
              || (data.grid_u_l1 > 50.0f)
              || (data.pv_total_w > 0);
    if (data.valid) {
        // Load = PV + Grid(+import/-export) - Battery(+tölt/-kisüt)
        // bat_w pozitív = tölt (fogyaszt), negatív = kisüt (termel)
        // grid_w pozitív = import (fogyaszt), negatív = export (termel)
        data.load_w_total = data.pv_total_w + data.grid_w_total - data.bat_w;
        if (data.load_w_total < 0) data.load_w_total = 0;

        Serial.printf("[Deye] LOAD (számított): %.0fW = PV(%+.0f) + Grid(%+.0f) - Bat(%+.0f)\n",
                      data.load_w_total, data.pv_total_w, data.grid_w_total, data.bat_w);
        data.calcBalance();
        _ok++;
    }

    return data.valid;
}
