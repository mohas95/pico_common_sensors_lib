# Pico Common Sensor Lib

This is a library for compiling and interfacing with many off the shelf sensors with the raspberry pi pico, usually readily available for the arduino platform 

Sensors that have been intergrated into this library:
- DFRobot EC analog sensor
- DFRobot PH (v2.0) analog sensor
- DFRobot A02YYUW ultrasonic sensor (uart)
- k30 CO2 Sensor (i2c)
- DS18B20 temperature sensor (onewire)
- SHT35 temperature and humidity sensor (i2c)

## Using library in project

To pull this library to your c++ project:

```bash
git clone git@github.com:mohas95/pico_common_sensors_lib.git
```
then in your CMakeLists.txt add: 
```cmake
add_subdirectory(<path to library>/pico_common_sensors_lib)

target_link_libraries(my_project pico_common_sensors_lib)
```



### Example usage
#### DFROBOT EC Example
```cpp
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

```


#### DFROBOT PH Example
```cpp
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

```

#### DFROBOT A02YYUW Ultrasonic sensor

```cpp
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
```

#### DS18B20 ONEWIRE Temperature probe
```cpp
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
```

#### Combined I2C and Analog environmental Sensors
```cpp
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
```