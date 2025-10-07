#pragma once
#include "i2cSensor.h"
#include <map>
#include <string>
#include <cmath>
#include <stdio.h>

class OPT4048 : public I2CSensor {
public:
    OPT4048(i2c_inst_t* bus, uint8_t addr = 0x44,
            uint16_t read_delay_ms = 100,
            uint16_t sample_rate_ms = 1000)
        : I2CSensor(bus, addr, read_delay_ms, sample_rate_ms) {}

    void init() override {
        last_read_request_ = time_us_64();

        // --- Read Device ID ---
        uint16_t id = readRegister16(REG_DEVICE_ID);
        printf("OPT4048 Device ID: 0x%04X\n", id);
        if (id != 0x0821) {
            printf("⚠️  Unexpected Device ID (want 0x0821)\n");
        }

        // --- Configure continuous conversion mode ---
        uint16_t config = 0x0000;
        config |= (3u << 0);    // MODE = 3 (continuous)
        config |= (3u << 12);   // RANGE = 3 (mid)
        config |= (7u << 8);    // CONVERSION TIME = ~50ms
        writeRegister16(REG_CONFIGURATION, config);
        sleep_ms(100);

        // --- Start conversions explicitly (MODE = 3) ---
        uint16_t start = readRegister16(REG_CONFIGURATION);
        start |= 0x0003;
        writeRegister16(REG_CONFIGURATION, start);

        // --- Optional readback ---
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
        if (now - last_read_request_ < (uint64_t)sample_rate_ms_ * 1000ULL)
            return false;
        read_ready_ = true;
        last_read_request_ = now;
        last_time_since_ready_ = now;
        return true;
    }

    bool load_read_resp() override {
        if (!read_ready_) return false;
        if ((time_us_64() - last_time_since_ready_) <= (uint64_t)read_delay_ms_ * 1000ULL)
            return false;
        read_ready_ = false;

        // --- Read 18 bytes (2 status + 4x4 channel data) ---
        uint8_t cmd = REG_RESULTS;
        uint8_t buf[18];
        writeBytes(&cmd, 1, true);
        if (readBytes(buf, 18) != 18) {
            printf("❌ Failed to read OPT4048 data\n");
            return false;
        }

        // 🔍 Print raw data for debugging
        printf("Raw frame: ");
        for (int i = 0; i < 18; ++i) printf("%02X ", buf[i]);
        printf("\n");

        // Skip first 2 status bytes
        uint8_t *data = &buf[2];

        // --- Decode 4 channels (W, X, Y, Z) ---
        uint32_t raw[4] = {0};
        for (int ch = 0; ch < 4; ++ch) {
            uint8_t e_msb = data[4 * ch];       // exponent nibble + mantissa high nibble
            uint8_t m_mid = data[4 * ch + 1];
            uint8_t m_low = data[4 * ch + 2];
            uint8_t crc   = data[4 * ch + 3];   // low nibble = CRC (ignore)

            uint8_t exponent = e_msb >> 4;
            uint32_t mantissa =
                ((uint32_t)(e_msb & 0x0F) << 16) |
                ((uint32_t)m_mid << 8) |
                (uint32_t)m_low;

            mantissa &= 0xFFFFF; // 20-bit mantissa
            raw[ch] = mantissa << exponent;
        }

        float W = raw[0];
        float X = raw[1];
        float Y = raw[2];
        float Z = raw[3];

        // --- Derived values ---
        float sum = X + Y + Z;
        float cie_x = (sum > 0) ? (X / sum) : 0;
        float cie_y = (sum > 0) ? (Y / sum) : 0;
        float lux   = Y * 0.003f; // rough scaling
        float n = (cie_x - 0.3320f) / (0.1858f - cie_y);
        float cct = (449.0f * powf(n, 3)) + (3525.0f * powf(n, 2)) +
                    (6823.3f * n) + 5520.33f;

        // --- Store results ---
        all_data_["W_raw"] = W;
        all_data_["X_raw"] = X;
        all_data_["Y_raw"] = Y;
        all_data_["Z_raw"] = Z;
        all_data_["cie_x"] = cie_x;
        all_data_["cie_y"] = cie_y;
        all_data_["lux"]   = lux;
        all_data_["cct_K"] = cct;

        return true;
    }

    std::map<std::string, float> getData() override { return all_data_; }

private:
    // ---- Register map ----
    static constexpr uint8_t REG_RESULTS        = 0x05;
    static constexpr uint8_t REG_CONFIGURATION  = 0x0A;
    static constexpr uint8_t REG_STATUS         = 0x0C;
    static constexpr uint8_t REG_DEVICE_ID      = 0x11;

    std::map<std::string, float> all_data_;

    // ---- I²C helpers ----
    void writeRegister16(uint8_t reg, uint16_t value) {
        uint8_t data[3] = {
            reg,
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0xFF)
        };
        writeBytes(data, 3);
    }

    uint16_t readRegister16(uint8_t reg) {
        uint8_t cmd = reg;
        uint8_t buf[2] = {0};
        writeBytes(&cmd, 1, true);
        if (readBytes(buf, 2) != 2) return 0;
        return (buf[0] << 8) | buf[1];
    }
};

