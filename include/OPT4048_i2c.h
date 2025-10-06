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

            // Soft reset or configuration if needed
            // Write continuous conversion mode
            uint16_t config = 0x8000; // Continuous mode, defaults for range/time
            writeRegister16(REG_CONFIG, config);
            sleep_ms(50);
        }

        bool read() override {
            query_read_cmd();
            if (read_ready_) {
                bool resp = load_read_resp();
                return resp;
            }
            return false;
        }

        bool query_read_cmd() override {
            uint64_t now = time_us_64();
            if (now - last_read_request_ < sample_rate_ms_ * 1000) {
                return false;
            } else {
                // OPT4048 continuously measures once configured
                // no explicit trigger needed, just wait for read_delay_ms_
                read_ready_ = true;
                last_read_request_ = now;
                last_time_since_ready_ = now;
            }
            return true;
        }

        bool load_read_resp() override {
            if (read_ready_ && (time_us_64() - last_time_since_ready_) > read_delay_ms_ * 1000) {
                read_ready_ = false;

                uint32_t X = readRegister24(REG_RESULT_X);
                uint32_t Y = readRegister24(REG_RESULT_Y);
                uint32_t Z = readRegister24(REG_RESULT_Z);
                uint32_t IR = readRegister24(REG_RESULT_IR);

                // Basic normalization (raw data units)
                float sum = (float)(X + Y + Z);
                float x = sum > 0 ? X / sum : 0;
                float y = sum > 0 ? Y / sum : 0;

                // Approximate lux — TI datasheet provides calibration; here Y ≈ illuminance
                float lux = (float)Y * 0.003f;  // scale factor (empirical, adjust via calibration)

                // Approximate correlated color temperature (CCT)
                float n = (x - 0.3320f) / (0.1858f - y);
                float cct = (449.0f * powf(n, 3)) + (3525.0f * powf(n, 2)) + (6823.3f * n) + 5520.33f;

                // Store results
                all_data_["X_raw"] = X;
                all_data_["Y_raw"] = Y;
                all_data_["Z_raw"] = Z;
                all_data_["IR_raw"] = IR;
                all_data_["lux"] = lux;
                all_data_["cie_x"] = x;
                all_data_["cie_y"] = y;
                all_data_["cct_K"] = cct;

                return true;
            }
            return false;
        }

        std::map<std::string, float> getData() override { return all_data_; }

    private:
        // Register addresses
        static constexpr uint8_t REG_CONFIG    = 0x00;
        static constexpr uint8_t REG_RESULT_X  = 0x0D;
        static constexpr uint8_t REG_RESULT_Y  = 0x0E;
        static constexpr uint8_t REG_RESULT_Z  = 0x0F;
        static constexpr uint8_t REG_RESULT_IR = 0x10;
        static constexpr uint8_t REG_DEVICE_ID = 0x92;

        std::map<std::string, float> all_data_;

        // === I²C Helpers ===
        void writeRegister16(uint8_t reg, uint16_t value) {
            uint8_t data[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
            writeBytes(data, 3);
        }

        uint16_t readRegister16(uint8_t reg) {
            uint8_t cmd = reg;
            uint8_t buf[2];
            writeBytes(&cmd, 1, true);
            readBytes(buf, 2);
            return (buf[0] << 8) | buf[1];
        }

        uint32_t readRegister24(uint8_t reg) {
            uint8_t cmd = reg;
            uint8_t buf[3];
            writeBytes(&cmd, 1, true);
            readBytes(buf, 3);
            return (buf[0] << 16) | (buf[1] << 8) | buf[2];
        }
};