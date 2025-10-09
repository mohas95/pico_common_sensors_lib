#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include <stdio.h>
#include <string.h>
#include <sstream>

#include "SHT35_i2c.h"
#include "k30_i2c.h"
#include "o2ezo_i2c.h"
#include "bangbangController.h"
#include "AnalogSensor.h"

const uint8_t VALVE_PIN = 2;
const uint8_t I2C1_SDA_PIN = 6;
const uint8_t I2C1_SCL_PIN = 7;
const uint8_t I2C0_SDA_PIN = 0;
const uint8_t I2C0_SCL_PIN = 1;
const uint8_t ADC0_PAR_PIN = 26;

#define K30_ADDR 0x68
#define EZO_O2_ADDR 0x6C
#define SHT35_ADDR 0x44
const uint8_t adc_par_channel = 0;

void valve_on(){
    gpio_put(VALVE_PIN,1);
}

void valve_off(){
    gpio_put(VALVE_PIN,0);
}


void check_ezo_o2_output_mode() {
    const char *cmd = "O,?";
    i2c_write_blocking(i2c1, EZO_O2_ADDR,
                       (const uint8_t*)cmd, strlen(cmd), false);
    sleep_ms(300); // datasheet says 300 ms processing delay

    uint8_t resp[32] = {0};
    int res = i2c_read_blocking(i2c1, EZO_O2_ADDR, resp, sizeof(resp), false);

    if (res > 1) {
        // status code is resp[0], payload starts at resp[1]
        printf("Status code: %u\n", resp[0]);
        printf("O,? response: '%s'\n", &resp[1]);
    } else {
        printf("Failed to query O,? (res=%d)\n", res);
    }
}


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

auto parMapping = [](float voltage) -> std::map<std::string,float> {
    
    float par = voltage * 1000.0f ;
    
    return {{"par", par}};
};



int main(){
    stdio_init_all();
    adc_init();

    i2c_init(i2c1,100*1000);
    i2c_init(i2c0,100*1000);

    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    
    gpio_init(VALVE_PIN);
    gpio_set_dir(VALVE_PIN, GPIO_OUT);
    valve_off();

    AnalogSensor par_sensor(adc_par_channel, ADC0_PAR_PIN, parMapping);
    SHT35 sht(i2c0, 0x44);
    K30 k30(i2c1, 0x68);
    O2EZO o2ezo(i2c1, 0x6C);
    bangbangController CO2control(0.0, 100.0, 0.0, 10000.0);

    par_sensor.init();
    sht.init();
    k30.init();
    o2ezo.init();
    sleep_ms(1000);

    std::map<std::string,float> allData;
    std::string payload;

    uint64_t last_log = time_us_64();

    // Main loop
    while(true){

        CO2control.pollSerial();

        if(par_sensor.read()){
            auto par_data = par_sensor.getData();
            for (const auto &[key, value] : par_data) {
                allData[key] = value;
            }
        }

        if(sht.read()){
            auto sht_data = sht.getData();
            for (const auto &[key, value] : sht_data) {
                allData[key] = value; // overwrite if exists
            }
        }

        if(k30.read()){
            auto k30_data = k30.getData();
             for (const auto &[key, value] : k30_data) {
                allData[key] = value; // overwrite if exists
            }
        }    

        if(o2ezo.read()){
            auto o2ezo_data = o2ezo.getData();
            for (const auto &[key, value] : o2ezo_data) {
                allData[key] = value; // overwrite if exists
            }
        }

        if((time_us_64() - last_log) > 1 * 1000000){
            last_log = time_us_64();

            allData["setPoint_ppm"] = CO2control.getSetpoint(); // overwrite if exists
            allData["hystersis_ppm"] = CO2control.getHysteresis(); // overwrite if exists
            allData["valve_state"] = CO2control.getState(); // overwrite if exists

            if(CO2control.update(allData["co2_ppm"])){
                valve_on();
                // printf("Valve ON\n");
            } else{
                valve_off();
                // printf("Valve OFF\n");
            }

            payload = make_json_payload(allData);

            printf("%s\n", payload.c_str());

        }

        sleep_ms(1);

    }
}