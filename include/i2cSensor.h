#pragma once
#include "hardware/i2c.h"
#include <map>
#include <string>

class I2CSensor {
    protected:
        i2c_inst_t* bus_;
        uint8_t addr_;
        uint16_t read_delay_ms_;
        uint16_t sample_rate_ms_;
        uint64_t last_read_request_;
        uint64_t last_time_since_ready_;
        bool read_ready_;

    public:
        I2CSensor(i2c_inst_t* bus, uint8_t addr, uint16_t read_delay_ms, uint16_t sample_rate_ms) : bus_(bus), addr_(addr), read_delay_ms_(read_delay_ms), sample_rate_ms_(sample_rate_ms) {}
        

        virtual ~I2CSensor() = default;
        virtual void init() = 0;
        virtual bool read() = 0;
        virtual bool query_read_cmd() = 0;
        virtual bool load_read_resp() = 0;
        virtual std::map<std::string, float> getData() = 0;

        int writeBytes(const uint8_t *data, size_t len, bool nostop=false) {
            return i2c_write_blocking(bus_, addr_, data, len, nostop);
        }

        int readBytes(uint8_t *data, size_t len) {
            return i2c_read_blocking(bus_, addr_, data, len, false);
        }

        i2c_inst_t *bus() const { return bus_; }
        uint8_t address() const { return addr_; }

};