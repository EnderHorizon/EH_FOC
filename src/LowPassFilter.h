#pragma once

// ==========================
// 低通滤波器
// ==========================
class LowPassFilter
{
public:
    LowPassFilter(float Tf);
    ~LowPassFilter();
    // 重载（）方便调用
    float operator()(float x, float Ts);
    // 设置Tf系数
    void setTf(float Tf);

private:
    float Tf; // 时间常数
    float last_y;
};