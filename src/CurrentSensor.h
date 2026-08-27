#pragma once

#include "LowPassFilter.h"

class INA240A2
{
public:
    INA240A2();
    INA240A2(int pin, float resistor); // 引脚,定值电阻
    ~INA240A2();
    INA240A2(const INA240A2 &) = delete;

    // 初始化
    void init(int pin, float resistor);
    // 读取电流
    float getCurrent();

private:
    int pin;              // 引脚
    float resistor;       // 定值电阻
    float ratio;          // 系数
    int magnification;    // 电压放大倍数 INA240A2会将测得的U放大50倍
    float offset_voltage; // 失调电压
};
