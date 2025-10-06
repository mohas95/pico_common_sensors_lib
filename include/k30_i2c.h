#pragma once
#include "i2cSensor.h"
#include <stdio.h>


class K30 : public I2CSensor {
    public:
        K30(i2c_inst_t* bus, uint8_t addr=0x68, uint16_t read_delay_ms=10, uint16_t sample_rate_ms =1000 ) : I2CSensor(bus, addr, read_delay_ms, sample_rate_ms){}
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
                writeBytes(read_sensor_cmd_, 4);
                read_ready_ = true;
                last_read_request_ = now;
                last_time_since_ready_ = now;
            }
            return true;
        }

        bool load_read_resp() override{
            if(read_ready_ == true && (time_us_64() - last_time_since_ready_) > read_delay_ms_ * 1000){
                read_ready_ = false;
                uint8_t data[4];
                int res = readBytes(data, 4);
                if(res == 4){
                    uint16_t co2 = (data[1] << 8) | data[2];
                    co2_=(float)co2;
                    all_data_["co2_ppm"] = co2_;     
                } else{
                    printf("Failed to read K30 sensor, res=%d\n",res);
                    return false;
                }  
            }else{
                return false;
            }
            return true;
        }

        float co2() const { return co2_; }

        std::map<std::string, float> getData() override{ return all_data_; }

    private:
        float co2_ = 0.0f;
        uint8_t read_sensor_cmd_[4] = {0x22, 0x00, 0x08, 0x2A};
        std::map<std::string, float> all_data_;
};
