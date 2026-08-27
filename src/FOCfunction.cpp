#include "FOCfunction.h"

#include <Arduino.h>
#include <cmath>

// —— 常量 ——
static const float SQRT3_OVER_2 = sqrt(3) / 2; // sqrt(3)/2
static const float SQRT3_OVER_3 = sqrt(3) / 3; // sqrt(3)/3
static const uint8_t PWM_BITS = 8;             // PWM 分辨率 (bit)
static const uint32_t PWM_FREQ = 30000;        // PWM 频率 (Hz)
static const uint32_t PWM_MAX = 255;           // 最大占空比数值 (255)

Motor::Motor(int pwmA, int pwmB, int pwmC,
             int8_t pole_pairs, float voltage_power_supply)
    : pwmA(pwmA), pwmB(pwmB), pwmC(pwmC),

      pole_pairs(pole_pairs),

      voltage_power_supply(voltage_power_supply),
      shaft_angle(0), zero_electric_angle(0),

      Ualpha(0), Ubeta(0),
      Ua(0), Ub(0), Uc(0),
      dc_a(0), dc_b(0), dc_c(0),

      Ialpha(0), Ibeta(0),
      Ia(0), Ib(0), Ic(0),
      Iq(0), Id(0),

      Last_time(0)
{
    Kp = 0.133; //(voltage_power_supply / 2.0f) / 45.0f; // Kp系数
}

Motor::~Motor()
{
}

void Motor::initPWM()
{
    // 引脚模式
    pinMode(pwmA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(pwmC, OUTPUT);
    // LEDC 通道配置 (通道 0/1/2)
    ledcAttachPin(pwmA, 0);
    ledcAttachPin(pwmB, 1);
    ledcAttachPin(pwmC, 2);
    ledcSetup(0, PWM_FREQ, PWM_BITS);
    ledcSetup(1, PWM_FREQ, PWM_BITS);
    ledcSetup(2, PWM_FREQ, PWM_BITS);

    // 电机驱动使能信号
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);
}

// 初始化
void Motor::initEncoder(int GPIO_SDA, int GPIO_SCL)
{
    as5600.init(GPIO_SDA, GPIO_SCL, -1);
    as5600.setLowPassFilter(0.03f);
    // AS5600校准
    alignEncoder();
}

void Motor::initVelPID(float Kp, float Ki, float Kd, float ramp)
{
    vel_pid.init(Kp, Ki, Kd, ramp, voltage_power_supply / 2);
}

void Motor::initPosPID(float Kp, float Ki, float Kd, float ramp)
{
    pos_pid.init(Kp, Ki, Kd, ramp, voltage_power_supply / 2);
}

void Motor::initCurPID(float Kp, float Ki, float Kd, float ramp)
{
    cur_pid.init(Kp, Ki, Kd, ramp, voltage_power_supply / 2);
}

void Motor::initCurSen(int pinA, int pinB, float resistor)
{
    cur_sen[0].init(pinA, resistor); // pwmA
    cur_sen[1].init(pinB, resistor); // pwmB
}

float Motor::getElectricalAngle()
{
    return normalizeAngle((float)(as5600.getDIR() * pole_pairs) * as5600.getWrappedAngle() - zero_electric_angle);
}

void Motor::setPhaseVoltage(float Uq, float Ud, float electric_angle)
{
    // 限制
    Uq = Limitation(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);
    electric_angle = normalizeAngle(electric_angle);
    inversePark(Uq, Ud, electric_angle); // dq -> αβ
    inverseClark();                      // αβ -> abc
    setPWM();                            // 电压 -> PWM 输出
}

float Motor::getRealCurrent()
{
    Ia = cur_sen[0].getCurrent();
    Ib = cur_sen[1].getCurrent();
    Clark();
    Park();
    // Iq 是直流量，在这里低通滤波（相电流不滤波，见 CurrentSensor.cpp）
    unsigned long now_ts = micros();
    float Ts = (now_ts - Last_time) * 1e-6f;
    if (Ts < 0 || Ts > 0.3f)
    {
        Ts = 0.001f;
    }
    Last_time = now_ts;
    Iq = iq_filter(Iq, Ts);
    return Iq;
}

void Motor::alignEncoder()
{
    setPhaseVoltage(3, 0, 4.71238898038f);
    delay(3000);
    zero_electric_angle = getElectricalAngle();
    setPhaseVoltage(0, 0, 4.71238898038f);
}

void Motor::inversePark(float Uq, float Ud, float electric_angle)
{
    // 帕克逆变换
    float c = cosf(electric_angle);
    float s = sinf(electric_angle);
    Ualpha = Ud * c - Uq * s;
    Ubeta = -Ud * s + Uq * c;
}

void Motor::inverseClark()
{
    // 克拉克逆变换
    Ua = Ualpha + voltage_power_supply / 2;
    Ub = -0.5f * Ualpha + SQRT3_OVER_2 * Ubeta + voltage_power_supply / 2;
    Uc = -0.5f * Ualpha - SQRT3_OVER_2 * Ubeta + voltage_power_supply / 2;
}

void Motor::setPWM()
{
    // 电压 -> 占空比 [0, 1]
    dc_a = Limitation(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = Limitation(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = Limitation(Uc / voltage_power_supply, 0.0f, 1.0f);
    // 占空比 -> PWM 输出 (0~255)
    ledcWrite(0, dc_a * PWM_MAX);
    ledcWrite(1, dc_b * PWM_MAX);
    ledcWrite(2, dc_c * PWM_MAX);
}

void Motor::Park()
{
    float electric_angle = normalizeAngle(getElectricalAngle());
    float c = cosf(electric_angle);
    float s = sinf(electric_angle);
    // Id = c * Ia + s * Ib;
    Iq = -s * Ialpha + c * Ibeta;
}

void Motor::Clark()
{
    Ialpha = Ia;
    Ibeta = SQRT3_OVER_3 * Ia + SQRT3_OVER_3 * 2 * Ib;
}

float Motor::generateShaftAngle(float target_v)
{
    unsigned long now_time = micros();              // esp32开机运行时间 (微秒)
    float Ts = (now_time - Last_time) / 1000000.0f; // 时间间隔 (微秒)
    Last_time = now_time;
    // micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常
    if (Ts < 0)
    {
        Ts = 0.001f;
    }
    return normalizeAngle(shaft_angle + Ts * target_v);
}

void Motor::velocityOpenLoop(float target_v)
{
    shaft_angle = generateShaftAngle(target_v);
    float Uq = voltage_power_supply / 3;
    setPhaseVoltage(Uq, 0, getElectricalAngle());
}

void Motor::positionLoop(float target_p)
{
    // 计算误差,传入positionPID,再传入电流环
    currentLoop(pos_pid((float)(target_p - as5600.getDIR() * as5600.getUnwrappedAngle()) * 180 / PI));
}

void Motor::velocityLoop(float target_v)
{
    // 计算误差,传入velocityPID,再传入电流环
    currentLoop(vel_pid((float)(target_v - as5600.getDIR() * as5600.getVelocity()) * 180 / PI));
}

void Motor::currentLoop(float target_i)
{
    // 计算误差,传入currentPID
    setPhaseVoltage(cur_pid(target_i - getRealCurrent()), 0, getElectricalAngle());
}

float normalizeAngle(const float &angle)
{
    float a = fmod(angle, TWO_PI);
    return a >= 0 ? a : (a + TWO_PI);
}