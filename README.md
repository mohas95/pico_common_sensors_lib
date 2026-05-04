# Pico Common Sensor Lib

This is a library for compiling and interfacing with many off the shelf sensors, usually readily available for the arduino platform 




### DFROBOT PH API
CalState cal_state = CalState::Idle;

uint64_t cal_start_us = 0;
float last_voltage_mv = 0.0f;
float stable_voltage_mv = 0.0f;
int stable_count = 0;


void handle_command(const char* cmd) {
    if (strcmp(cmd, "calibration") == 0) {
        cal_state = CalState::Stabilizing;
        cal_start_us = time_us_64();
        stable_count = 0;
        last_voltage_mv = 0.0f;

        printf("Entering pH calibration mode...\n");
        printf("Waiting for voltage to stabilize...\n");
    }
}

void calibration_task(DFR_PH_Analog& ph, float voltage_mv) {
    if (cal_state != CalState::Stabilizing) {
        return;
    }

    float delta = fabsf(voltage_mv - last_voltage_mv);
    last_voltage_mv = voltage_mv;

    if (delta < 2.0f) {  // voltage changed less than 2 mV
        stable_count++;
    } else {
        stable_count = 0;
    }

    if (stable_count >= 20) {  // e.g. 20 stable loop samples
        stable_voltage_mv = voltage_mv;

        printf("Stable voltage detected: %.2f mV\n", stable_voltage_mv);

        bool ok = ph.calibrate(stable_voltage_mv);

        if (ok) {
            printf("Calibration saved.\n");
        } else {
            printf("Calibration failed.\n");
        }

        cal_state = CalState::Idle;
    }

    if (time_us_64() - cal_start_us > 30000000ULL) {
        printf("Calibration timeout.\n");
        cal_state = CalState::Idle;
    }
}