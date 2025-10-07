#pragma once
#include "i2cSensor.h"
#include <stdio.h>
#include <map>
#include <string>
#include <cmath>

class OPT4048 : public I2CSensor {
public:
    OPT4048(i2c_inst_t* bus, uint8_t addr = 0x44, uint16_t read_delay_ms = 100, uint16_t sample_rate_ms = 1000)
        : I2CSensor(bus, addr, read_delay_ms, sample_rate_ms) {}

    void init() override {
        last_read_request_ = time_us_64();

        // Step 1: Software reset
        writeRegister16(REG_CONTROL, 0x8000);
        sleep_ms(10);

        // Step 2: Verify Device ID
        uint16_t id = readRegister16(REG_DEVICE_ID);
        printf("OPT4048 Device ID: 0x%04X\n", id);
        if (id != 0x0190) {
            printf("⚠️  Warning: Unexpected Device ID. Check wiring or address.\n");
        }

        // Step 3: Configure measurement settings (continuous mode, 100ms conversion, medium range)
        uint16_t config = 0;
        config |= (1 << 15);  // Continuous conversion mode
        config |= (4 << 9);   // Integration time (approx 100ms)
        config |= (3 << 12);  // Range (medium)
        config |= (1 << 6);   // 2x averaging
        writeRegister16(REG_CONFIGURATION, config);
        sleep_ms(50);

        // Optional: Read back config
        uint16_t cfg_read = readRegister16(REG_CONFIGURATION);
        printf("OPT4048 Config readback: 0x%04X\n", cfg_read);
    }

    bool read() override {
        query_read_cmd();
        if (read_ready_) {
            return load_read_resp();
        }
        return false;
    }

    bool query_read_cmd() override {
        uint64_t now = time_us_64();
        if (now - last_read_request_ < sample_rate_ms_ * 1000) {
            return false;
        }
        read_ready_ = true;
        last_read_request_ = now;
        last_time_since_ready_ = now;
        return true;
    }

    bool load_read_resp() override {
        if (read_ready_ && (time_us_64() - last_time_since_ready_) > read_delay_ms_ * 1000) {
            read_ready_ = false;

            // Step 1: Optional status check (bit 7 = new data)
            uint16_t status = readRegister16(REG_STATUS);
            if (!(status & 0x0080)) {
                // No new data yet
                return false;
            }

            // Step 2: Read XYZ and IR results (24-bit each)
            uint32_t X  = readRegister24(REG_RESULT_X);
            uint32_t Y  = readRegister24(REG_RESULT_Y);
            uint32_t Z  = readRegister24(REG_RESULT_Z);
            uint32_t IR = readRegister24(REG_RESULT_IR);

            // Step 3: Normalize and compute derived values
            float sum = (float)(X + Y + Z);
            float x = sum > 0 ? X / sum : 0;
            float y = sum > 0 ? Y / sum : 0;

            // Lux is approximated from Y (per Adafruit approach)
            float lux = (float)Y * 0.003f;

            // Correlated Color Temperature (CCT) from McCamy's formula
            float n = (x - 0.3320f) / (0.1858f - y);
            float cct = (449.0f * powf(n, 3)) + (3525.0f * powf(n, 2)) + (6823.3f * n) + 5520.33f;

            all_data_["X_raw"]  = X;
            all_data_["Y_raw"]  = Y;
            all_data_["Z_raw"]  = Z;
            all_data_["IR_raw"] = IR;
            all_data_["lux"]    = lux;
            all_data_["cie_x"]  = x;
            all_data_["cie_y"]  = y;
            all_data_["cct_K"]  = cct;

            return true;
        }
        return false;
    }

    std::map<std::string, float> getData() override { return all_data_; }

private:
    // === Register Map (from Adafruit_OPT4048.h) ===
    static constexpr uint8_t REG_CONFIGURATION = 0x00;
    static constexpr uint8_t REG_CONTROL       = 0x01;
    static constexpr uint8_t REG_STATUS        = 0x02;
    static constexpr uint8_t REG_RESULT_X      = 0x0D;
    static constexpr uint8_t REG_RESULT_Y      = 0x0E;
    static constexpr uint8_t REG_RESULT_Z      = 0x0F;
    static constexpr uint8_t REG_RESULT_IR     = 0x10;
    static constexpr uint8_t REG_DEVICE_ID     = 0x92;

    std::map<std::string, float> all_data_;

    // === I²C Helpers ===
    void writeRegister16(uint8_t reg, uint16_t value) {
        uint8_t data[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
        writeBytes(data, 3);
    }

    uint16_t readRegister16(uint8_t reg) {
        uint8_t cmd = reg;
        uint8_t buf[2] = {0};
        writeBytes(&cmd, 1, true);
        readBytes(buf, 2);
        return (buf[0] << 8) | buf[1];
    }

    uint32_t readRegister24(uint8_t reg) {
        uint8_t cmd = reg;
        uint8_t buf[3] = {0};
        writeBytes(&cmd, 1, true);
        readBytes(buf, 3);
        return (buf[0] << 16) | (buf[1] << 8) | buf[2];
    }
};
