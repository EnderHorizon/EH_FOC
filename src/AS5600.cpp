#include "AS5600.h"
#include <Arduino.h>
#include <Wire.h>

constexpr int ADDRESS = 0x36;     // I2C 地址
constexpr int RAW_ANGLE_H = 0x0C; // 原始角度高字节寄存器地址
constexpr int RAW_ANGLE_L = 0x0D; // 原始角度低字节寄存器地址
constexpr int ZPOS_H = 0x01;      // 零点高字节寄存器地址
constexpr int ANGLE_H = 0x0E;     // 放缩后角度高字节寄存器地址

// ———— 常量 ————
constexpr float RAW_TO_RADIAN = 0.08789 * PI / 180; //(360.0f / 4096.0f) * (PI / 180.0f);

AS5600::AS5600()
    : GPIO_SDA(-1),
      GPIO_SCL(-1),
      last_angle(0.0f), turns(0), DIR(1),
      last_ts(0.0f), val_last_unwrapped(0.0f), lowpassfilter(0.01f)
{
}

AS5600::AS5600(int GPIO_SDA, int GPIO_SCL, int8_t DIR)
    : GPIO_SDA(GPIO_SDA),
      GPIO_SCL(GPIO_SCL),
      last_angle(0.0f), turns(0), DIR(DIR),
      last_ts(0.0f), val_last_unwrapped(0.0f), lowpassfilter(0.01f)
{
}

AS5600::~AS5600()
{
}

void AS5600::init(int GPIO_SDA, int GPIO_SCL, int8_t DIR)
{
    this->GPIO_SDA = GPIO_SDA;
    this->GPIO_SCL = GPIO_SCL;
    this->DIR = DIR;
    initI2C();
}

void AS5600::initI2C()
{
    Wire.begin(GPIO_SDA, GPIO_SCL, 400000UL); // 设置 I2C 时钟频率为 400kHz
    delay(1000);                              // 等待传感器稳定
}

uint16_t AS5600::readRegister16(uint8_t reg_h)
{
    Wire.beginTransmission(ADDRESS); // 告诉I2C控制器，AS5600的地址
    Wire.write(reg_h);               // 寄存器地址放入 I2C 发送缓冲区
    Wire.endTransmission(false);     // 发送缓冲区数据，并保持总线连接
    Wire.requestFrom(ADDRESS, 2);    // 寄存器的2bite数据依次放到 SDA 线上供主机读取
    while (Wire.available() < 2)     // 检查I2C接收缓冲区里已经收到多少个bite
    {
    } // 等待数据接收完成
    uint8_t high = Wire.read(); // [B11][B10][B9][B8][B7][B6][B5][B4]|[B3][B2][B1][B0][0][0][0][0]
    uint8_t low = Wire.read();  //               high                |            low

    return (uint16_t)(high << 8) | low;
}

uint16_t AS5600::getRawAngle()
{
    return readRegister16(RAW_ANGLE_H);
}

uint16_t AS5600::readZeroPosition()
{
    return 0;
}

void AS5600::updateAngle()
{
    float now_angle = getRawAngle() * RAW_TO_RADIAN;
    float d_angle = now_angle - last_angle; // 计算与上次角度差值
    if (abs(d_angle) > (0.8f * TWO_PI))
    {
        turns += (d_angle > 0) ? -1 : 1;
    }
    // last_ts = micros();
    last_angle = now_angle;
}

float AS5600::getWrappedAngle()
{
    updateAngle();
    return last_angle; // 转化为弧度制
}

float AS5600::getUnwrappedAngle()
{
    updateAngle();
    return (TWO_PI * (float)turns) + last_angle;
}

int8_t AS5600::getDIR()
{
    return DIR;
}

float AS5600::getVelocity()
{
    float now_ts = micros();
    float val_now_unwrapped = getUnwrappedAngle();
    float Ts = (now_ts - last_ts) * 1e-6f;
    if (Ts < 0)
    {
        Ts = 0.001f;
    }
    float val = (val_now_unwrapped - val_last_unwrapped) / Ts; // 包含圈数(任意角)

    last_ts = now_ts;
    val_last_unwrapped = val_now_unwrapped;

    return lowpassfilter(val, Ts);
}

void AS5600::setLowPassFilter(float Tf)
{
    lowpassfilter.setTf(Tf);
}
