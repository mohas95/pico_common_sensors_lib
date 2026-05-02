#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>
#include <sstream>

#include "DS18B20_onewire.h"

const uint8_t ONEWIRE_PIN = 1;


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
    gpio_init(ONEWIRE_PIN);

    DS18B20_onewire temp_sensor(ONEWIRE_PIN);
    temp_sensor.init();
    sleep_ms(1000);

    std::map<std::string,float> allData;
    std::string payload;


    uint64_t last_log = time_us_64();


    // Main Loop
    while(true){

        if(temp_sensor.read()){
            auto temp_data = temp_sensor.getData();
            for (const auto &[key,value] : temp_data){
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