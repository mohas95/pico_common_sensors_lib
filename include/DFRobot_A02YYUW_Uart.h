#pragma once

#include <cstdint>
#include <cstdio>
#include <map>
#include <string>


#include "pico/stdlib.h"
#include "hardware/uart.h"


class DFR_A02YYUW_Ultrasonic{
    public:
        DFR_A02YYUW_Ultrasonic(uart_inst_t* uart_id, uint32_t timeout_ms= 200, uint16_t sample_rate_ms =1000): uart_id_(uart_id),
                                                                                                               timeout_us_(timeout_ms*1000ULL),
                                                                                                                sample_rate_us_(sample_rate_ms*1000ULL),
                                                                                                                distance_mm_(0.0f),
                                                                                                                last_read_request_(0){}
    
    
        void init(){
            uart_set_format(uart_id_, 8,1, UART_PARITY_NONE);
            uart_set_fifo_enabled(uart_id_, true);

            last_read_request_ = time_us_64();
        };

        bool read(){

            return query_uart();
        }


        bool query_uart(){
            uint8_t byte = 0;
            uint64_t start = time_us_64();

            if (start - last_read_request_ < sample_rate_us_){
                clear_uart_buffer_();
                return false;
            }else{
                while(time_us_64()-start < timeout_us_){
                    if (!readByte_(byte)) {
                        continue;
                    }

                    if (byte != HEADER_BYTE) {
                        continue;
                    }

                    uint8_t data[4];
                    data[0]=byte;

                    if(!readN_(&data[1],3)){
                        last_read_request_ = time_us_64();
                        return false;
                    }

                    uint8_t checksum = data[0] + data[1] + data[2];

                    if (checksum!= data[3]){
                        last_read_request_ = time_us_64();
                        return false;
                    }

                    distance_mm_ = static_cast<float>((data[1] << 8) | data[2]);

                    all_data_["distance_mm"] = distance_mm_;
                    last_read_request_ = time_us_64();


                    return true;
                }
            }

            last_read_request_ = time_us_64();
            return false;

        }

        std::map<std::string, float> getData() const{return all_data_;}


    private:
        static constexpr uint8_t HEADER_BYTE = 0xFF;

        uart_inst_t* uart_id_;
        uint64_t timeout_us_;
        uint64_t sample_rate_us_;
        float distance_mm_ = 0.0f;
        std::map<std::string, float> all_data_;

        uint64_t last_read_request_;

        bool readByte_(uint8_t& out){
            if (uart_is_readable(uart_id_)){
                out = uart_getc(uart_id_);

                return true;
            }

            return false;
        }


        bool readN_(uint8_t* buffer, size_t len){
            size_t count = 0;
            uint64_t start = time_us_64();

            while (count<len){
                if(uart_is_readable(uart_id_)){
                    buffer[count++] = uart_getc(uart_id_);
                }

                if(time_us_64() - start > timeout_us_){
                    return false;
                }
            }

            return true;
        }

        void clear_uart_buffer_(){
            uint64_t start = time_us_64();

            while (uart_is_readable(uart_id_)){
                uart_getc(uart_id_);

                if (time_us_64() - start > 2000) {  // 2 ms max
                    break;
                }
            }
        }

};