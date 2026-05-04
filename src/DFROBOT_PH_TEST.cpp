#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>
#include <sstream>

#include "AnalogSensor.h"
#include "DFRobot_PH_Analog.h"
#include "DFRobot_CalibrationManager.h"

const uint8_t ADC1_PH_PIN = 27;
const uint8_t adc_ph_channel = 1;
#define CMD_BUFFER_SIZE 64

bool read_serial_command(char* out_cmd) {
    static char buffer[CMD_BUFFER_SIZE];
    static int index = 0;

    int ch = getchar_timeout_us(0);  // non-blocking read

    if (ch == PICO_ERROR_TIMEOUT) {
        return false;
    }

    // User pressed Enter
    if (ch == '\n' || ch == '\r') {
        if (index == 0) {
            return false;
        }

        buffer[index] = '\0';

        strcpy(out_cmd, buffer);

        index = 0;
        memset(buffer, 0, sizeof(buffer));

        return true;
    }

    // Normal typed character
    if (index < CMD_BUFFER_SIZE - 1) {
        buffer[index++] = (char)ch;
    }

    return false;
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


DFR_PH_Analog ph_converter;
DFR_CalibrationManager ph_cal(ph_converter);


auto phMapping = [](float voltage) -> std::map<std::string,float> {
    
    float voltage_mv = voltage * 1000.0f ;

    float ph = ph_converter.readPH(voltage_mv);
    
    return {{"ph", ph},{"ph_voltage_mv", voltage_mv}};
};


int main(){
    stdio_init_all();
    adc_init();


    sleep_ms(1000);

    ph_converter.begin();

    AnalogSensor ph_sensor(adc_ph_channel, ADC1_PH_PIN, phMapping);
    ph_sensor.init();

    sleep_ms(1000);

    std::map<std::string,float> allData;
    std::string payload;


    uint64_t last_log = time_us_64();


    // Main Loop
    while(true){

        char command[64];
        if (read_serial_command(command)) {
            printf("Command received: %s\n", command);
            ph_cal.handleCommand(command);
        }

        if(ph_sensor.read()){
            auto ph_data = ph_sensor.getData();
            for (const auto &[key,value] : ph_data){
                allData[key] = value;
            }

            ph_cal.task(ph_data["ph_voltage_mv"]);

        }

        if((time_us_64() - last_log) > 1 * 1000000){
            last_log = time_us_64();

            payload = make_json_payload(allData);
            printf("%s\n", payload.c_str());

            if (ph_cal.isCalibrating()) {
                printf("Calibrating PH");
            }
        
        }


        sleep_ms(1);

    }

}