#pragma once
#include "i2cSensor.h"
#include <stdio.h>
#include <stdlib.h>


class O2EZO : public I2CSensor {
    public:
        O2EZO(i2c_inst_t* bus, uint8_t addr=0x6C, uint16_t read_delay_ms=900, uint16_t sample_rate_ms =1000) :  I2CSensor(bus, addr, read_delay_ms, sample_rate_ms){}
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
                writeBytes(read_sensor_cmd_, 1);
                read_ready_ = true;
                last_read_request_ = now;
                last_time_since_ready_ = now;
            }
            return true;
        }

        bool load_read_resp() override{
            if(read_ready_ == true && (time_us_64() - last_time_since_ready_) > read_delay_ms_ * 1000){
                read_ready_ = false;
                uint8_t resp[32];
                memset(resp, 0, sizeof(resp));

                int res = readBytes(resp, 32);

                uint8_t code = resp[0];

                switch (code) {
                    case 1: break; // success
                    case 2: printf("Status: Syntax error\n"); return false;
                    case 254: printf("Status: Still processing\n"); return false;
                    case 255: printf("Status: No data\n"); return false;
                    default: printf("Status: Unknown (%u)\n", code); return false;
                }
                
                if (res > 1) {
                    // Copy ASCII payload (skip status byte)
                    char payload[31];
                    int payload_len = res - 1;
                    if (payload_len > 30) payload_len = 30;
                    memcpy(payload, &resp[1], payload_len);
                    payload[payload_len] = '\0';

                    // Convert to float if numeric
                    if ((payload[0] >= '0' && payload[0] <= '9') || payload[0] == '-' || payload[0] == '+') {
                        o2_ = atof(payload);
                        all_data_["o2_%"] = o2_; 
                    }
                } else {
                    printf("Failed to read EZO O2 sensor (res=%d)\n", res);
                    return false;
                }
            }else{
                return false;
            }
            return true;
        }

        float o2() const { return o2_; }

        std::map<std::string, float> getData() override{ return all_data_; }

    private:
        float o2_ = 0.0f;
        const uint8_t* read_sensor_cmd_ = (const uint8_t*)"R";
        std::map<std::string, float> all_data_;
};
