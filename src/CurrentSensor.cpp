#include "CurrentSensor.h"

#include <Arduino.h>

INA240A2::INA240A2()
    : pin(0), resistor(0.0f), ratio(0.0f), magnification(1), offset_voltage(0.0f)
{
}

INA240A2::INA240A2(int pin, float resistor)
    : pin(pin), resistor(resistor), ratio(0.0f), magnification(1), offset_voltage(0.0f)
{
}

INA240A2::~INA240A2()
{
}

void INA240A2::init(int pin, float resistor)
{
    this->pin = pin;
    this->resistor = resistor;
    this->magnification = 50;
    this->ratio = 1.0f / resistor / magnification;
    // 测失调电压
    for (int i = 0; i < 1000; ++i)
    {
        offset_voltage += analogRead(this->pin);
    }
    offset_voltage = offset_voltage / 1000.0f;
    //Serial.println(offset_voltage);
}

float INA240A2::getCurrent()
{
    return ((analogRead(pin) - offset_voltage) * 3.3f / 4095.0f) * ratio;
}
