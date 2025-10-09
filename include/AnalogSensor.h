#pragma once

#include <functional>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <cstdio>

class AnalogSensor {
    public:
        AnalogSensor(uint8_t adc_channel, uint8_t pin, std::function<float(float)> mapFn, float vref =0, uint32_t sample_rate_ms =500 ) : 
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
            if (now - last_time_since_read_ < sample_rate_ms_ *1000){
                return false;
            }else{
                adc_select_input(adc_channel_);
                raw_ = adc_read();
                voltage_ = (raw * VREF) / (1 << 12);


                return true;
            }

        }

        bool getData(){

        }



        
    private:
        const float VREF_;
        const uint8_t pin_;
        const uint8_t adc_channel_;
        std::function<float(float)> mapping_func_;

        uint16_t raw_;
        float voltage_;

        uint64_t last_time_since_read_;
        uint32_t sample_rate_ms_;

};