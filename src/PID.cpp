#include "PID.h"

#include <Arduino.h>

PID::PID()
    : Kp(0.0f), Ki(0.0f), Kd(0.0f), proportional(0.0f), integral(0.0f), derivative(0.0f), output_ramp(0.0f),
      last_integral(0.0f), last_error(0.0f), limit(0.0f), last_ts(0), last_output(0)
{
}

PID::PID(float Kp, float Ki, float Kd, float ramp, float limit)
    : Kp(Kp), Ki(Ki), Kd(Kd), proportional(0.0f), integral(0.0f), derivative(0.0f), output_ramp(ramp),
      last_integral(0.0f), last_error(0.0f), limit(limit), last_ts(0), last_output(0)
{
}

void PID::init(float Kp, float Ki, float Kd, float ramp, float limit)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->output_ramp = ramp;
    this->limit = limit;
    last_ts = micros();
}

PID::~PID()
{
}

float PID::operator()(float error)
{
    float now_ts = micros();
    float Ts = (now_ts - last_ts) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;
    // P环
    proportional = Kp * error;
    // I环(integral = ∫edt)
    integral = last_integral + Ki * Ts * (error + last_error) * 0.5f;
    integral = Limitation(integral, -limit, limit);
    // D环
    derivative = Kd * (error - last_error) / Ts;

    float output = proportional + integral + derivative;
    output = Limitation(output, -limit, limit);
    //缓坡
    if (output_ramp > 0)
    {
        float output_rate = (output - last_output) / Ts;
        if (output_rate > output_ramp)
        {
            output = last_output + output_ramp * Ts;
        }
        else if (output_rate < -output_ramp)
        {
            output = last_output - output_ramp * Ts;
        }
    }

    last_output = output;
    last_integral = integral;
    last_error = error;
    last_ts = now_ts;

    return output;
}