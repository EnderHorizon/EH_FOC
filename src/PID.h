#pragma once

#define Limitation(amt, low, high) (amt < low ? low : (amt > high ? high : amt));
// ==========================
//           PID
// ==========================
class PID
{
public:
    PID();
    PID(float Kp, float Ki, float Kd, float ramp, float limit);
    ~PID();
    PID(const PID &) = delete;

    // 初始化/设置 PID 参数（setup() 中调用）
    void init(float Kp, float Ki, float Kd, float ramp, float limit);
    float operator()(float error);

private:
    float proportional;
    float integral;
    float derivative;
    float Kp;
    float Ki;
    float Kd;
    float output_ramp;

    float last_integral;
    float last_error;
    float last_ts;
    float last_output;
    float limit;
};