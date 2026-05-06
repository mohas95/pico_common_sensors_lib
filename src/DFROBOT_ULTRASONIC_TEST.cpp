#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>
#include <sstream>

#include "DFRobot_A02YYUW_Uart.h"

#define UART_ID uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5

std::string make_json_payload(const std::map<std::string,float>& data){
    std::ostringstream ss;
    ss << "{";
    bool first = true;

    for (const auto &[key, value] : data){
        if (!first) ss<< ", ";
        ss << "\"" << key << "\": " << value;
        first = false;
    }

    ss << "}";

    return ss.str();
}

int main(){
    stdio_init_all();

    uart_init(UART_ID,9600);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    DFR_A02YYUW_Ultrasonic ultrasonic_sensor(UART_ID);
    ultrasonic_sensor.init();
    sleep_ms(1000);

    std::map<std::string,float> allData;
    std::string payload;


    uint64_t last_log = time_us_64();


    // Main Loop
    while(true){

        if(ultrasonic_sensor.read()){
            auto ultrasonic_data = ultrasonic_sensor.getData();
            for (const auto &[key,value] : ultrasonic_data){
                allData[key] = value;
            }
        }

        if((time_us_64() - last_log) > 1 * 1000000){
            last_log = time_us_64();

            payload = make_json_payload(allData);
            printf("%s\n", payload.c_str());
        
        
        
        }

        sleep_ms(1);

    }

}