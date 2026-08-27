#pragma once
#include <cstdint> // int8_t / uint16_t / int64_t
#include "LowPassFilter.h"

// ==========================
// AS5600
// ==========================
class AS5600
{
public:
    AS5600();
    AS5600(int GPIO_SDA, int GPIO_SCL, int8_t DIR);
    ~AS5600();
    AS5600(const AS5600 &) = delete;            // 禁止拷贝构造
    AS5600 &operator=(const AS5600 &) = delete; // 禁止拷贝赋值

    // 初始化引脚与 I2C（setup() 中调用）
    void init(int GPIO_SDA, int GPIO_SCL, int8_t DIR);
    // 初始化 I2C
    void initI2C();
    // 读取零点值（0~4095）
    uint16_t readZeroPosition();
    // 更新角度
    void updateAngle();
    // 读取角度值（0~2π）
    float getWrappedAngle();
    // 读取角度值（任意角）
    float getUnwrappedAngle();
    // 获取编码器方向
    int8_t getDIR();
    // 获取速度
    float getVelocity();
    // 设置低通滤波系数
    void setLowPassFilter(float Tf);

private:
    // 读取寄存器地址中16位数据
    uint16_t readRegister16(uint8_t reg_h);
    // 读取原始角度值（0~4095）
    uint16_t getRawAngle();

    int GPIO_SDA;
    int GPIO_SCL;
    float last_angle;
    int64_t turns;
    int8_t DIR;

    float last_ts; //(micros)
    float val_last_unwrapped;
    LowPassFilter lowpassfilter; // 低通滤波器
};