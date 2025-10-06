#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "i2cSensor.h"


class bangbangController {
    public:
        bangbangController(float setpoint, float hysteresis, float min_val, float max_val)
            : setpoint_(setpoint), hysteresis_(hysteresis), min_val_(min_val), max_val_(max_val), state_(false) {}



        void setSetpoint(float sp)    { setpoint_ = sp; }
        void setHysteresis(float h)   { hysteresis_ = h; }
        void setMinVal(float minv)    { min_val_ = minv; }
        void setMaxVal(float maxv)    { max_val_ = maxv; }
        float getSetpoint() const     { return setpoint_; }
        float getHysteresis() const   { return hysteresis_; }
        bool getState() const { return state_; }

        bool update(float current_value) {
            if (current_value < setpoint_ - hysteresis_) {
                state_ = true; // Turn on
            } else if (current_value > setpoint_ + hysteresis_) {
                state_ = false; // Turn off
            }
            return state_;
        }


        void handleSerialCommand(const char *input_line) {
            char cmd[32];
            float value;
            if (sscanf(input_line, "%31s %f", cmd, &value) == 2) {
                if (strcmp(cmd, "setpoint") == 0) {
                    setpoint_ = value;
                    printf("Setpoint updated to %.2f\n", setpoint_);
                } else if (strcmp(cmd, "hysteresis") == 0) {
                    hysteresis_ = value;
                    printf("Hysteresis updated to %.2f\n", hysteresis_);
                } else if (strcmp(cmd, "min") == 0) {
                    min_val_ = value;
                    printf("Min value updated to %.2f\n", min_val_);
                } else if (strcmp(cmd, "max") == 0) {
                    max_val_ = value;
                    printf("Max value updated to %.2f\n", max_val_);
                } else {
                    printf("Unknown command: %s\n", cmd);
                }
            } else {
                printf("Invalid command format. Use: <param> <value>\n");
            }
        }

        void pollSerial() {
            static char buffer[64];
            static int idx = 0;

            int c = getchar_timeout_us(0); // non-blocking
            if (c != PICO_ERROR_TIMEOUT) {
                if (c == '\n' || c == '\r') {
                    if (idx > 0) {
                        buffer[idx] = '\0';
                        handleSerialCommand(buffer);
                        idx = 0; // reset for next line
                    }
                } else if (idx < (int)sizeof(buffer) - 1) {
                    buffer[idx++] = (char)c;
                }
            }
        }
        

    private:
        float setpoint_;
        float hysteresis_;
        float min_val_;
        float max_val_;
        bool state_;
};