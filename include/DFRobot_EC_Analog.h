#pragma once
#include <cstdint>
#include <cstring>
#include <cctype>
#include <cstdio>
#include <stdio.h>

#include "DFRobot_AnalogSensor.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"


#define EC_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - 18 * FLASH_SECTOR_SIZE) //validate that this does not conflict with other library flash storage
#define EC_MAGIC 0x45434341  // "ECCA"

#define EC_RES2 820.0f
#define EC_REF 200.0f

struct ECCalibrationData {
    uint32_t magic;
    float k_value_low;
    float k_value_high;
    uint32_t checksum;
};

class DFR_EC_Analog : public DFR_AnalogSensor {
    public:
        DFR_EC_Analog(bool clear_cal = false, const char* label="EC") : raw_ec_(0.0f),
                                                                        ec_value_(0.0f),
                                                                        k_value_(1.0f),
                                                                        k_value_low_(1.0f),
                                                                        k_value_high_(1.0f),
                                                                        label_(label){
            if (clear_cal){
                clear_cal_();
            }                                                                                                                                                                                    
        }

        void begin(){
            if (!loadCalibration_()) {
                k_value_low_ = 1.0f;
                k_value_high_ = 1.0f;
                printf("%s failed to load. using defaults k_low: %.4f, k_high: %.4f\n", label_, k_value_low_, k_value_high_);

                // saveCalibration_();
            }

            k_value_ = k_value_low_;

        }

        float readEC(float voltage_mv, float temperature_c = 25.0f){
            raw_ec_ = 1000.0f * voltage_mv / EC_RES2 / EC_REF;
            
            float value_temp = raw_ec_ * k_value_;

            if (value_temp>2.5f){
                k_value_ = k_value_high_;
            } else if (value_temp<2.0f){
                k_value_ = k_value_low_;
            }

            value_temp = raw_ec_*k_value_;
            ec_value_ = value_temp / (1.0f + 0.0185f * (temperature_c - 25.0f));

            return ec_value_;
            
        }

        bool calibrate(float voltage_mv, float temperature_c = 25.0f) override{

            raw_ec_ = 1000.0f * voltage_mv / EC_RES2 / EC_REF;

            float comp_ec_solution = 0.0f;

            if (raw_ec_ > 0.9f && raw_ec_ < 1.9f) {
                // 1413 us/cm = 1.413 ms/cm
                comp_ec_solution = 1.413f * (1.0f + 0.0185f * (temperature_c - 25.0f));
            } else if (raw_ec_ > 9.0f && raw_ec_ < 16.8f) {
                // 12.88 ms/cm
                comp_ec_solution = 12.88f * (1.0f + 0.0185f * (temperature_c - 25.0f));
            } else {
                printf("%s: calibration failed. Raw EC %.3f not recognized.\n", label_, raw_ec_);
                return false;
            }

            float k_temp = EC_RES2 * EC_REF * comp_ec_solution / 1000.0f / voltage_mv;

            if (k_temp < 0.5f || k_temp > 1.5f) {
                printf("%s: calibration failed. K %.4f out of range.\n", label_, k_temp);
                return false;
            }

            if (raw_ec_ > 0.9f && raw_ec_ < 1.9f) {
                k_value_low_ = k_temp;
                printf("%s: low calibration saved. K_low: %.4f\n", label_, k_value_low_);
            } else {
                k_value_high_ = k_temp;
                printf("%s: high calibration saved. K_high: %.4f\n", label_, k_value_high_);
            }

            saveCalibration_();
            return true;
        }

        const char* label() const override{ return label_;}
        
    private:
        float raw_ec_; 
        float k_value_;
        float k_value_low_;
        float k_value_high_;
        float ec_value_;
        const char* label_;

        uint32_t checksum_(const ECCalibrationData& data){
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
            uint32_t sum = 0;

            for (size_t i = 0; i < sizeof(ECCalibrationData) - sizeof(uint32_t); i++) {
                sum += bytes[i];
            }

            return sum;
        }
        
        bool loadCalibration_() {
            const ECCalibrationData* stored =
                reinterpret_cast<const ECCalibrationData*>(XIP_BASE + EC_FLASH_OFFSET);

            if (stored->magic != EC_MAGIC) {
                return false;
            }

            if (stored->checksum != checksum_(*stored)) {
                return false;
            }

            k_value_low_ = stored->k_value_low;
            k_value_high_ = stored->k_value_high;

            printf("%s calibration loaded. k_low: %.4f, k_high: %.4f\n", label_, k_value_low_, k_value_high_);


            return true;
        }
 
        void saveCalibration_() {
            ECCalibrationData data;
            data.magic = EC_MAGIC;
            data.k_value_low = k_value_low_;
            data.k_value_high = k_value_high_;
            data.checksum = checksum_(data);

            uint8_t buffer[FLASH_SECTOR_SIZE];
            memset(buffer, 0xFF, FLASH_SECTOR_SIZE);
            memcpy(buffer, &data, sizeof(data));

            uint32_t interrupts = save_and_disable_interrupts();

            flash_range_erase(EC_FLASH_OFFSET, FLASH_SECTOR_SIZE);
            flash_range_program(EC_FLASH_OFFSET, buffer, FLASH_SECTOR_SIZE);

            restore_interrupts(interrupts);
            printf("%s: calibration saved to flash. k_low: %.4f, k_high: %.4f\n", label_, k_value_low_, k_value_high_);

        }

        void clear_cal_() {
            uint32_t interrupts = save_and_disable_interrupts();
            flash_range_erase(EC_FLASH_OFFSET, FLASH_SECTOR_SIZE);
            restore_interrupts(interrupts);
            printf("%s: calibration cleared from flash.\n", label_);
        }

};