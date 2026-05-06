#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>
#include <sstream>

#include "AnalogSensor.h"
#include "DFRobot_EC_Analog.h"
#include "DFRobot_CalibrationManager.h"

const uint8_t ADC2_EC_PIN = 28;
const uint8_t adc_ec_channel = 2;
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


DFR_EC_Analog ec_converter;
DFR_CalibrationManager ec_cal(ec_converter);


auto ecMapping = [](float voltage) -> std::map<std::string,float> {
    
    float voltage_mv = voltage * 1000.0f ;

    float ec = ec_converter.readEC(voltage_mv);
    
    return {{"ec_ms/cm", ec},{"ec_voltage_mv", voltage_mv}};
};


int main(){
    stdio_init_all();
    adc_init();


    sleep_ms(1000);

    ec_converter.begin();

    AnalogSensor ec_sensor(adc_ec_channel, ADC2_EC_PIN, ecMapping);
    ec_sensor.init();

    sleep_ms(1000);

    std::map<std::string,float> allData;
    std::string payload;


    uint64_t last_log = time_us_64();


    // Main Loop
    while(true){

        char command[64];
        if (read_serial_command(command)) {
            printf("Command received: %s\n", command);
            ec_cal.handleCommand(command);
        }

        if(ec_sensor.read()){
            auto ec_data = ec_sensor.getData();
            for (const auto &[key,value] : ec_data){
                allData[key] = value;
            }

            ec_cal.task(ec_data["ec_voltage_mv"]);

        }

        if((time_us_64() - last_log) > 1 * 1000000){
            last_log = time_us_64();

            payload = make_json_payload(allData);
            printf("%s\n", payload.c_str());

            if (ec_cal.isCalibrating()) {
                printf("Calibrating EC");
            }
        
        }


        sleep_ms(1);

    }

}