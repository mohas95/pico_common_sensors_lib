#pragma once
#include "onewireSensor.h"
#include <map>
#include <string>
#include <stdio.h>
#include <cmath>




class DS18B20_onewire : public OneWireSensor{

    public:
        DS18B20_onewire(uint8_t ow_pin, uint16_t read_delay_ms=750, uint16_t sample_rate_ms =1000) : OneWireSensor(ow_pin, read_delay_ms, sample_rate_ms){}

        void init() override {
            last_read_request_ = time_us_64();
            ow_release_();
        }

        bool read() override{
            if(!read_ready_){
                query_read_cmd();
                return false;
            }
             
            return load_read_resp();
            
        }

        bool query_read_cmd() override{
            uint64_t now = time_us_64();

            if(now - last_read_request_ < sample_rate_ms_ * 1000){
                return false;
            }else{

                if (!ow_reset_()) {
                    printf("DS18B20 not found\n");
                    return false;
                }

                ow_write_byte_(0xCC); // Skip ROM, only one sensor on bus
                ow_write_byte_(0x44); // Convert T

                read_ready_ = true;
                last_read_request_ = now;
                last_time_since_ready_ = now;
            }
            return true;
        }


        bool load_read_resp() override{
            if(read_ready_ == true && (time_us_64() - last_time_since_ready_) > read_delay_ms_ * 1000){
                read_ready_ = false;


                if (!ow_reset_()) {
                    printf("DS18B20 Error\n");
                    temp_c_=NAN;
                    all_data_["temp_c"]= NAN;
                    return true;
                }

                ow_write_byte_(0xCC); // Skip ROM
                ow_write_byte_(0xBE); // Read scratchpad

                uint8_t temp_lsb = ow_read_byte_();
                uint8_t temp_msb = ow_read_byte_();

                int16_t raw_temp = static_cast<int16_t>((temp_msb << 8) | temp_lsb);

                temp_c_ = raw_temp / 16.0f;   
                all_data_["temp_c"] = temp_c_;            

            }else{
                return false;
            }
            return true;
        }


        float temp_c() const { return temp_c_; }
        std::map<std::string, float> getData() override{ return all_data_; }

    private:
        float temp_c_ = 0.0f;
        std::map<std::string, float> all_data_;


};