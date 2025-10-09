#pragma once

#include <functional>
#include <cstdio>
#include <map>
#include <tuple>
#include <string>

#include "pico/stdlib.h"
#include "hardware/adc.h"



class AnalogSensor {
    public:
        AnalogSensor(uint8_t adc_channel, uint8_t pin, std::function<std::map<std::string, float>(float)> mapFn, float vref =3.3f, uint32_t sample_rate_ms =500 ) : 
                                                                            adc_channel_(adc_channel),
                                                                            pin_(pin),
                                                                            VREF_(vref),
                                                                            sample_rate_ms_(sample_rate_ms),
                                                                            mapping_func_(mapFn){}

        void init(){
            adc_gpio_init(pin_);
            last_time_since_read_= time_us_64();
        }

        bool read(){
            uint64_t now = time_us_64();
            if (now - last_time_since_read_ < sample_rate_ms_ *1000ULL){
                return false;
            }else{
                adc_select_input(adc_channel_);
                raw_ = adc_read();
                voltage_ = (raw_ * VREF_) / (1 << 12);

                all_data_ =  mapping_func_(voltage_);

                last_time_since_read_ = now;

                return true;
            }

        }

        const std::map<std::string, float>& getData(){return all_data_;}


        
    private:
        const float VREF_;
        const uint8_t pin_;
        const uint8_t adc_channel_;
        std::function<std::map<std::string, float>(float)> mapping_func_;
        std::map<std::string, float> all_data_;

        uint16_t raw_;
        float voltage_;

        uint64_t last_time_since_read_;
        uint32_t sample_rate_ms_;

};