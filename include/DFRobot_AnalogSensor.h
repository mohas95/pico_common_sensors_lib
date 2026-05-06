#pragma once
#include <cstdint>
#include <cstdio>

class DFR_AnalogSensor{
    public:
    virtual ~DFR_AnalogSensor()=default;
    virtual bool calibrate(float voltage_mv, float temperature_c = 25.0f) = 0;
    virtual const char* label() const = 0;
    
};