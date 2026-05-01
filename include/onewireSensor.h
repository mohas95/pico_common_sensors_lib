#pragma once
#include <map>
#include <string>
#include "pico/stdlib.h"
#include "hardware/gpio.h"


class OneWireSensor{
    protected:
        const uint8_t ow_pin_;
        uint16_t read_delay_ms_;
        uint16_t sample_rate_ms_;
        uint64_t last_read_request_;
        uint64_t last_time_since_ready_;
        bool read_ready_;

        void ow_drive_low_(){
            gpio_set_dir(ow_pin_,GPIO_OUT);
            gpio_put(ow_pin_, 0);
        }

        void ow_release_(){
            gpio_set_dir(ow_pin_, GPIO_IN);
        }

        bool ow_read_pin_(){
            return gpio_get(ow_pin_);
        }

        
        bool ow_reset_(){
            ow_drive_low_();
            sleep_us(480);
            ow_release_();
            sleep_us(70);

            bool presence = !ow_read_pin_();

            sleep_us(410);

            return presence;
        }

        void ow_write_bit_(bool bit){
            if (bit){
                ow_drive_low_();
                sleep_us(6);
                ow_release_();
                sleep_us(64);
            }else{
                ow_drive_low_();
                sleep_us(60);
                ow_release_();
                sleep_us(10);
            }
        }

        bool ow_read_bit_(){
            ow_drive_low_();
            sleep_us(6);
            ow_release_();
            sleep_us(9);

            bool bit = ow_read_pin_();
            sleep_us(55);

            return bit;
        }

        void ow_write_byte_(uint8_t byte){
            for (int i = 0; i<8; i++){
                ow_write_bit_(byte & 0x01);
                byte >>=1;
            }
        }

        uint8_t ow_read_byte_(){
            uint8_t byte = 0;

            for (int i = 0; i<8; i++){
                if(ow_read_bit_()){
                    byte |= (1<<i); 
                }
            }
            return byte;
        }
       
    public:
        OneWireSensor(uint8_t ow_pin, uint16_t read_delay_ms, uint16_t sample_rate_ms) : ow_pin_(ow_pin),
                                                                                         read_delay_ms_(read_delay_ms), 
                                                                                         sample_rate_ms_(sample_rate_ms),
                                                                                         last_read_request_(0), last_time_since_ready_(0),
                                                                                         read_ready_(false) {}
        virtual ~OneWireSensor() = default;
        virtual void init() = 0;
        virtual bool read() = 0;
        virtual bool query_read_cmd() = 0;
        virtual bool load_read_resp() = 0;
        virtual std::map<std::string, float> getData() = 0;

};