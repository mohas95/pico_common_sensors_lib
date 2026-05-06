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



#define PH_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - 17 * FLASH_SECTOR_SIZE) //validate that this does not conflict with other library flash storage
#define PH_MAGIC 0x50484341  // "PHCA"


struct CalibrationData{
    uint32_t magic;
    float neutral_voltage;
    float acid_voltage;
    uint32_t checksum;

};

enum class CalState {
    Idle,
    Stabilizing
};



class DFR_PH_Analog : public DFR_AnalogSensor {
    public:
        DFR_PH_Analog(bool clear_cal = false, float acid_low_thresh=1720.0f, float acid_high_thresh=2210.0f, float neutral_low_thresh= 1200.0f, float neutral_high_thresh=1678.0f, const char* label="PH") :  acid_low_thresh_(acid_low_thresh),
                                                                                                                                                                                                              acid_high_thresh_(acid_high_thresh),
                                                                                                                                                                                                              neutral_low_thresh_(neutral_low_thresh),
                                                                                                                                                                                                              neutral_high_thresh_(neutral_high_thresh),
                                                                                                                                                                                                              ph_value_(7.0f),
                                                                                                                                                                                                              acid_voltage_(2032.44f),
                                                                                                                                                                                                              neutral_voltage_(1500.0f),
                                                                                                                                                                                                              temperature_(25.0f),
                                                                                                                                                                                                              label_(label){

            if (clear_cal){
                clear_cal_();
            }
        }


        void begin(){

            if(!loadCalibration_()){
                neutral_voltage_ = 1500.0f;
                acid_voltage_ = 2032.44f;
                // saveCalibration_();

            printf("failed to load using defaults - neutral_cal: %.2f, acid_cal: %.2f\n", neutral_voltage_, acid_voltage_);


            }

        }
        
        float readPH(float voltage_mv, float temperature_c=25.0){
            temperature_ = temperature_c;

            float slope =(7.0f - 4.0f) / ((neutral_voltage_ - 1500.0f) / 3.0f - (acid_voltage_ - 1500.0f) / 3.0f);
            
            float temp_coeff = (temperature_c + 273.15f) / 298.15f;

            slope *= temp_coeff;
            
            float intercept = 7.0f - slope * (neutral_voltage_ - 1500.0f) / 3.0f;

            ph_value_ = slope * (voltage_mv - 1500.0f) / 3.0f + intercept;
        
            return ph_value_;
        }
        
        bool calibrate(float voltage_mv, float temperature_c=25.0) override {
            if (voltage_mv > neutral_low_thresh_ && voltage_mv < neutral_high_thresh_) {
                neutral_voltage_ = voltage_mv;
                saveCalibration_();

                printf("pH 7 calibration saved: %.2f mV\n", neutral_voltage_);
                return true;
            }

            if (voltage_mv > acid_low_thresh_&& voltage_mv < acid_high_thresh_) {
                acid_voltage_ = voltage_mv;
                saveCalibration_();

                printf("pH 4 calibration saved: %.2f mV\n", acid_voltage_);
                return true;
            }

            printf("Voltage %.2f mV not recognized as pH 7 or pH 4 buffer.\n", voltage_mv);
            return false;
        }

        float neutralVoltage() const {return neutral_voltage_;}
        float acidVoltage() const {return acid_voltage_;}
        const char* label() const override {return label_;}

    private:

        float ph_value_;
        float acid_voltage_;
        float neutral_voltage_;
        float voltage_;
        float temperature_;
        float acid_low_thresh_;
        float acid_high_thresh_;
        float neutral_low_thresh_;
        float neutral_high_thresh_;
        const char* label_;


        uint32_t checksum_(const CalibrationData& data){
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
            uint32_t sum =0;

            for (size_t i =0; i<sizeof(CalibrationData) - sizeof(uint32_t); i++){
                sum+= bytes[i];
            }

            return sum;
        }

        bool loadCalibration_(){
            const CalibrationData* stored = reinterpret_cast<const CalibrationData*>(XIP_BASE + PH_FLASH_OFFSET);
            
            if (stored->magic != PH_MAGIC){
                return false;
            }

            if (stored->checksum != checksum_(*stored)){
                return false;
            }

            neutral_voltage_ = stored->neutral_voltage;
            acid_voltage_ = stored->acid_voltage;

            printf("calibration loaded sucessfully - neutral_cal: %.2f, acid_cal: %.2f\n", neutral_voltage_, acid_voltage_);


            return true;
        
        }

        void saveCalibration_(){
            CalibrationData data;
            data.magic = PH_MAGIC;
            data.neutral_voltage = neutral_voltage_;
            data.acid_voltage = acid_voltage_;
            data.checksum = checksum_(data);

            uint8_t buffer[FLASH_SECTOR_SIZE];

            memset(buffer,0xFF, FLASH_SECTOR_SIZE);
            memcpy(buffer, &data, sizeof(data));

            uint32_t interrupts = save_and_disable_interrupts();

            flash_range_erase(PH_FLASH_OFFSET, FLASH_SECTOR_SIZE);
            flash_range_program(PH_FLASH_OFFSET, buffer, FLASH_SECTOR_SIZE);

            restore_interrupts(interrupts);
            printf("calibration saved to flash- neutral_cal: %.2f, acid_cal: %.2f\n", neutral_voltage_, acid_voltage_);

        }

        void clear_cal_() {
            uint32_t interrupts = save_and_disable_interrupts();
            flash_range_erase(PH_FLASH_OFFSET, FLASH_SECTOR_SIZE);
            restore_interrupts(interrupts);
        }


};