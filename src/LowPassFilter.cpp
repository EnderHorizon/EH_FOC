#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float Tf)
    : Tf(Tf), last_y(0.0f)
{
}

LowPassFilter::~LowPassFilter()
{
}

float LowPassFilter::operator()(float x, float Ts)
{
    if (Ts < 0.0f)
    {
        Ts = 1e-3f;
    }
    else if (Ts > 0.3f)
    {
        last_y = x;
        return x; // 时间间隔太长，直接返回原本值
    }

    float alpha = Tf / (Tf + Ts);
    float y = alpha * last_y + (1.0f - alpha) * x;
    last_y = y;
    return y;
}

void LowPassFilter::setTf(float Tf)
{
    this->Tf = Tf;
}
