#pragma once
#include "i2cSensor.h"
#include <stdio.h>


class SHT35 : public I2CSensor {
    public:
        SHT35(i2c_inst_t* bus, uint8_t addr=0x44, uint16_t read_delay_ms=20, uint16_t sample_rate_ms =1000 ) :  I2CSensor(bus, addr, read_delay_ms, sample_rate_ms) {}
        void init() override{
            last_read_request_ = time_us_64();
        }
        
        bool read() override{
            query_read_cmd();
            if(read_ready_){
                bool resp = load_read_resp();
                return resp;
            }
            return false;
        }

        bool query_read_cmd() override{
            uint64_t now = time_us_64();

            if(now - last_read_request_ < sample_rate_ms_ * 1000){
                return false;
            }else{
                writeBytes(read_sensor_cmd_, 2);
                read_ready_ = true;
                last_read_request_ = now;
                last_time_since_ready_ = now;
            }
            return true;
        }

        bool load_read_resp() override{

            if(read_ready_ == true && (time_us_64() - last_time_since_ready_) > read_delay_ms_ * 1000){
                read_ready_ = false;
                uint8_t data[6];
                
                if (readBytes(data, 6) != 6) {
                    printf("Failed to read SHT35 sensor\n");
                    return false;
                }

                uint16_t rawTemp = (data[0] << 8) | data[1];
                uint16_t rawHum  = (data[3] << 8) | data[4];

                temperature_ = -45.0f + 175.0f * ((float)rawTemp / 65535.0f);
                humidity_ = 100.0f * ((float)rawHum / 65535.0f);

                all_data_["temperature_C"] = temperature_;
                all_data_["humidity_%"]    = humidity_;
            }else{
                return false;
            }

            return true;

        }

        float temperature() const { return temperature_; }
        float humidity() const { return humidity_; }

        std::map<std::string, float> getData() override{ return all_data_; }

    private:
        float temperature_ = 0.0f;
        float humidity_ = 0.0f;
        uint8_t read_sensor_cmd_[2] = {0x24, 0x00};
        std::map<std::string, float> all_data_;
};