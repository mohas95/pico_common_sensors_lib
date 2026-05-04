#pragma once


#include <cstdio>
#include <cstring>
#include <cmath>

#include "pico/stdlib.h"
#include "DFRobot_PH_Analog.h"


class DFR_CalibrationManager{
    public:
        enum class State{
            Idle,
            Stabilizing
        };

        DFR_CalibrationManager(DFR_PH_Analog& ph_sensor, float stable_thresh_mv=10.0f, uint16_t stable_samples_req=20, uint32_t timeout_ms = 30000) : ph_(ph_sensor),
                                                                                                                                                    stable_thresh_mv_(stable_thresh_mv),
                                                                                                                                                    stable_samples_req_(stable_samples_req),
                                                                                                                                                    timeout_us_(timeout_ms *1000ULL) {}
        
        void handleCommand(const char* cmd){
            if(strcmp(cmd, "calibration") == 0){
                start();
            }
        }

        void start(){
            state_ = State::Stabilizing;
            start_time_us_ = time_us_64();
            last_voltage_mv_ = NAN;
            stable_count_ = 0;

            printf("Entering pH calibration mode\n");
            printf("Place probe in pH 7 or pH 4 buffer solution\n");
            printf("Waiting for voltage to stabilize...\n");
        }

        void task(float voltage_mv){
            if (state_!= State::Stabilizing){
                return;
            }

            if(!std::isnan(last_voltage_mv_)){
                float delta = fabsf(voltage_mv - last_voltage_mv_);
                if (delta<stable_thresh_mv_){
                    stable_count_++;
                }else{
                    stable_count_ = 0;
                }
            }

            last_voltage_mv_ = voltage_mv;

            if(stable_count_>= stable_samples_req_){
                finish(voltage_mv);
                return;
            }

            if (time_us_64() - start_time_us_ > timeout_us_){
                printf("Calibration timeout\n");
                state_ = State::Idle;
            }

        }

        bool isCalibrating() const {
            return state_ == State::Stabilizing;
        }
        
    
    private:
        DFR_PH_Analog& ph_;

        State state_ = State::Idle;

        float stable_thresh_mv_;
        uint16_t stable_samples_req_;
        uint64_t timeout_us_;

        uint64_t start_time_us_ = 0;
        float last_voltage_mv_ = NAN;
        uint16_t stable_count_ = 0;

        void finish(float voltage_mv){
            printf("Stable voltage detected: %.2f mV\n", voltage_mv);

            bool ok = ph_.calibrate(voltage_mv);

            if(ok){
                printf("pH calibration saved\n");
                printf("neutral_cal: %.2f, acid_cal: %.2f\n",
                   ph_.neutralVoltage(),
                   ph_.acidVoltage());
            }else{
                printf("pH calibration failed\n");
            }
            state_ = State::Idle;
        }

};