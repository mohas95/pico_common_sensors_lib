#pragma once
#include "i2cSensor.h"
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <stdio.h>

/*
 * OPT4048 Pico driver (Adafruit-like flow)
 * - Uses your I2CSensor base (writeBytes / readBytes, with repeated-start support)
 * - Channel names: X, Y, Z, W (W = wideband/clear)
 * - Default I2C addr: 0x44 (ADDR high -> 0x45)
 */

class OPT4048 : public I2CSensor {
public:
    // Tunables you can pass in
    OPT4048(i2c_inst_t* bus, uint8_t addr = 0x44,
            uint16_t read_delay_ms = 100, uint16_t sample_rate_ms = 1000)
        : I2CSensor(bus, addr, read_delay_ms, sample_rate_ms) {}

    // ----- Public API -----
    void init() override {
        last_read_request_ = time_us_64();

        // 1) Software reset, then clear it
        writeRegister16(REG_CONTROL, CONTROL_SW_RESET);
        sleep_ms(10);
        writeRegister16(REG_CONTROL, 0x0000);
        sleep_ms(5);

        // 2) Read Device ID for sanity
        uint16_t id = readRegister16(REG_DEVICE_ID);
        printf("OPT4048 Device ID: 0x%04X\n", id);   // Adafruit lib expects 0x0821
        if (id != 0x0821) {
            printf("⚠️  Unexpected Device ID (wanted 0x0821). Check power/address.\n");
        }

        // 3) Configure continuous mode, mid gain, ~50ms conv time, light averaging
        uint16_t cfg = 0;
        cfg |= CFG_MODE_CONT;     // continuous conversions
        cfg |= (3u << CFG_RANGE_SHIFT);   // range = 3 (mid)
        cfg |= (7u << CFG_CONVT_SHIFT);   // conv time index ~50ms (see note)
        cfg |= (1u << CFG_AVG_SHIFT);     // small averaging (2x)
        writeRegister16(REG_CONFIGURATION, cfg);
        sleep_ms(100); // let first integration complete

        // Optional: read back config for debug
        uint16_t cfg_rb = readRegister16(REG_CONFIGURATION);
        printf("OPT4048 Config readback: 0x%04X\n", cfg_rb);
    }

    bool read() override {
        query_read_cmd();
        if (read_ready_) return load_read_resp();
        return false;
    }

    bool query_read_cmd() override {
        const uint64_t now = time_us_64();
        if (now - last_read_request_ < (uint64_t)sample_rate_ms_ * 1000ULL) return false;
        read_ready_ = true;
        last_read_request_ = now;
        last_time_since_ready_ = now;
        return true;
    }

    bool load_read_resp() override {
        if (!read_ready_) return false;
        if ((time_us_64() - last_time_since_ready_) <= (uint64_t)read_delay_ms_ * 1000ULL) return false;
        read_ready_ = false;

        // (Optional) You can poll a data-ready bit here if you like.
        // Many apps just read; the result registers hold the latest conversion.

        // Read 24-bit results MSB..LSB
        const uint32_t X = readRegister24(REG_RESULT_X);
        const uint32_t Y = readRegister24(REG_RESULT_Y);
        const uint32_t Z = readRegister24(REG_RESULT_Z);
        const uint32_t W = readRegister24(REG_RESULT_W);

        // Derived quantities
        const double sum = static_cast<double>(X) + static_cast<double>(Y) + static_cast<double>(Z);
        const double cie_x = (sum > 0.0) ? static_cast<double>(X) / sum : 0.0;
        const double cie_y = (sum > 0.0) ? static_cast<double>(Y) / sum : 0.0;

        // Lux: Adafruit treats Y as illuminance proxy; scale is empirical & depends on range/time.
        // Start with a small scale and calibrate later against a lux reference.
        const double lux = static_cast<double>(Y) * 0.003; // tweak per your setup

        // McCamy CCT (quick, good-enough)
        const double n = (cie_x - 0.3320) / (0.1858 - cie_y);
        const double cct = (449.0 * n * n * n) + (3525.0 * n * n) + (6823.3 * n) + 5520.33;

        all_data_["X_raw"] = static_cast<float>(X);
        all_data_["Y_raw"] = static_cast<float>(Y);
        all_data_["Z_raw"] = static_cast<float>(Z);
        all_data_["W_raw"] = static_cast<float>(W);
        all_data_["cie_x"] = static_cast<float>(cie_x);
        all_data_["cie_y"] = static_cast<float>(cie_y);
        all_data_["lux"]   = static_cast<float>(lux);
        all_data_["cct_K"] = static_cast<float>(cct);

        return true;
    }

    std::map<std::string, float> getData() override { return all_data_; }

    // --- Optional helpers you can call later from your app ---
    void setRange(uint8_t range_sel) {         // 0..7 typically
        uint16_t cfg = readRegister16(REG_CONFIGURATION);
        cfg &= ~(0x7u << CFG_RANGE_SHIFT);
        cfg |= (static_cast<uint16_t>(range_sel & 0x7u) << CFG_RANGE_SHIFT);
        writeRegister16(REG_CONFIGURATION, cfg);
    }
    void setConvTimeIdx(uint8_t idx) {         // conv time index, see datasheet (0..11 possible on OPT4048)
        uint16_t cfg = readRegister16(REG_CONFIGURATION);
        cfg &= ~(0x7u << CFG_CONVT_SHIFT);     // keep 3-bit field here; long times may be handled by an extended bit elsewhere per datasheet
        cfg |= (static_cast<uint16_t>(idx & 0x7u) << CFG_CONVT_SHIFT);
        writeRegister16(REG_CONFIGURATION, cfg);
    }
    void setAveraging(uint8_t avg_idx) {       // 0..3 typical
        uint16_t cfg = readRegister16(REG_CONFIGURATION);
        cfg &= ~(0x3u << CFG_AVG_SHIFT);
        cfg |= (static_cast<uint16_t>(avg_idx & 0x3u) << CFG_AVG_SHIFT);
        writeRegister16(REG_CONFIGURATION, cfg);
    }

private:
    // ---- Registers (matching Adafruit names/addresses) ----
    static constexpr uint8_t REG_CONFIGURATION = 0x00;
    static constexpr uint8_t REG_CONTROL       = 0x01;
    static constexpr uint8_t REG_STATUS        = 0x02;
    static constexpr uint8_t REG_RESULT_X      = 0x0D;
    static constexpr uint8_t REG_RESULT_Y      = 0x0E;
    static constexpr uint8_t REG_RESULT_Z      = 0x0F;
    static constexpr uint8_t REG_RESULT_W      = 0x10; // wideband/clear (Adafruit calls it W)
    static constexpr uint8_t REG_DEVICE_ID     = 0x92;

    // ---- Bit fields (kept aligned with Adafruit’s usage) ----
    static constexpr uint16_t CONTROL_SW_RESET = 0x8000;     // write then clear

    static constexpr uint16_t CFG_MODE_CONT    = 1u << 15;   // continuous conversions
    static constexpr uint8_t  CFG_RANGE_SHIFT  = 12;         // 3 bits
    static constexpr uint8_t  CFG_CONVT_SHIFT  = 9;          // 3 bits (short list); datasheet has extended times too
    static constexpr uint8_t  CFG_AVG_SHIFT    = 6;          // 2 bits

    std::map<std::string, float> all_data_;

    // ---- I2C helpers ----
    void writeRegister16(uint8_t reg, uint16_t value) {
        uint8_t data[3] = { reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF) };
        writeBytes(data, 3);
    }
    uint16_t readRegister16(uint8_t reg) {
        uint8_t cmd = reg;
        uint8_t buf[2] = {0, 0};
        writeBytes(&cmd, 1, /*repeat start*/ true);
        if (readBytes(buf, 2) != 2) return 0;
        return static_cast<uint16_t>(buf[0] << 8) | buf[1];
    }
    uint32_t readRegister24(uint8_t reg) {
        uint8_t cmd = reg;
        uint8_t buf[3] = {0, 0, 0};
        writeBytes(&cmd, 1, /*repeat start*/ true);
        if (readBytes(buf, 3) != 3) return 0;
        return (static_cast<uint32_t>(buf[0]) << 16) |
               (static_cast<uint32_t>(buf[1]) << 8)  |
               (static_cast<uint32_t>(buf[2]));
    }
};
