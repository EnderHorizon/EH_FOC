#pragma once

#include "AS5600.h"
#include "PID.h"
#include "CurrentSensor.h"

#define Limitation(amt, low, high) (amt < low ? low : (amt > high ? high : amt));

// ==========================
// 电机 FOC 控制类
// ==========================
class Motor
{
public:
    // 构造函数: PWM引脚, AS5600引脚, 电压限制(V), 极对数, 母线电压(V)
    Motor(int pwmA, int pwmB, int pwmC,
          int8_t pole_pairs, float voltage_power_supply);
    ~Motor();
    Motor(const Motor &) = delete;            // 禁止拷贝构造
    Motor &operator=(const Motor &) = delete; // 禁止拷贝赋值
    Motor(Motor &&) = delete;                 // 禁止移动构造
    Motor &operator=(Motor &&) = delete;      // 禁止移动赋值

    // 初始化
    void initPWM();
    void initEncoder(int GPIO_SDA, int GPIO_SCL);
    void initVelPID(float Kp, float Ki, float Kd, float ramp);
    void initPosPID(float Kp, float Ki, float Kd, float ramp);
    void initCurPID(float Kp, float Ki, float Kd, float ramp);
    void initCurSen(int pinA, int pinB, float resistor);

    // 电角度 = 机械角度 × 极对数 - 零点偏移
    float getElectricalAngle();
    // 机械角度读写
    float getShaftAngle() const { return shaft_angle; }
    void setShaftAngle(float angle) { shaft_angle = angle; }
    // 速度开环控制(rad/s)
    void velocityOpenLoop(float target_v);

    // 位置闭环控制（rad）
    void positionLoop(float target_p);
    // 速度闭环控制
    void velocityLoop(float target_v);
    // 电流闭环控制
    void currentLoop(float target_i);
    
    // 传感三相电流得到实际Iq, Id
    float getRealCurrent();

private:
    // Uq->三相电
    // 逆帕克变换: dq -> αβ
    void inversePark(float Uq, float Ud, float electric_angle);
    // 逆克拉克变换: αβ -> abc 三相电压
    void inverseClark();
    // 电压 -> 占空比 -> PWM 输出
    void setPWM();

    // Iq <- 三相电流（电流传感器）
    // 帕克变换：abc -> αβ
    void Park();
    // 克拉克变换: αβ -> dq
    void Clark();

    // 机械角度生成器（开环时使用）
    float generateShaftAngle(float target_v);
    // 设置 dq 轴电压并输出三相 PWM
    void setPhaseVoltage(float Uq, float Ud, float electric_angle); // Uq: q轴电压(V), Ud: d轴电压(V), electric_angle: 电角度(rad)

    // AS5600校准
    void alignEncoder();

    // —— 硬件参数 ——
    int pwmA, pwmB, pwmC;       // PWM 引脚
    int8_t pole_pairs;          // 极对数
    float voltage_power_supply; // 母线供电电压 (V)

    // —— 运行状态 ——
    float shaft_angle;         // 机械角度 (rad)
    unsigned long Last_time;   // 上次调用时间 (micros)
    float zero_electric_angle; // 电角度零点偏移 (rad)，校准后使用

    // —— 中间量 ——
    float Ualpha, Ubeta;    // αβ 轴电压
    float Ua, Ub, Uc;       // 三相电压
    float dc_a, dc_b, dc_c; // 三相占空比

    float Ialpha, Ibeta; // αβ 轴电流
    float Ia, Ib, Ic;    // 三相电流
    float Iq, Id;

    // —— 系数 ——
    float Kp;

public:
    // —— 编码器AS5600 ——
    AS5600 as5600;
    // —— PID ——
    PID vel_pid;
    PID pos_pid;
    PID cur_pid;
    // —— CurrentSenor ——
    INA240A2 cur_sen[2];             // 分别为ab相,c相电流可算
    LowPassFilter iq_filter{0.05f}; // Iq 低通滤波器
};

// 角度归一化到 [0, 2π)
float normalizeAngle(const float &angle);