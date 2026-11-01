#include "PID.h"
#include <Arduino.h>

template <typename T>
static inline T clamp(T val, T low, T high)
{
    if (val < low)
        val = low;
    else if (val > high)
        val = high;

    return val;
};

PID_Controller::PID_Controller()
    : kp(0.0f), ki(0.0f), kd(0.0f),
      output_ramp_limit(0.0f), output_limit(0.0f),
      error_prev(0.0f), output_prev(0.0f), integral_prev(0.0f),
      timestamp_prev(0), has_sample(false)
{
}

PID_Controller::PID_Controller(float kp, float ki, float kd, float ramp_limit, float output_limit)
    : kp(kp), ki(ki), kd(kd), output_ramp_limit(ramp_limit), output_limit(output_limit),
      error_prev(0.0f), output_prev(0.0f), integral_prev(0.0f),
      timestamp_prev(0), has_sample(false)
{
}

void PID_Controller::configure(float kp, float ki, float kd, float ramp_limit, float output_limit)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->output_ramp_limit = ramp_limit;
    this->output_limit = output_limit;
    if (output_limit != 0)
    {
        output_prev = clamp(output_prev, -output_limit, output_limit);
        integral_prev = clamp(integral_prev, -output_limit, output_limit);
    }
}

float PID_Controller::update(float error)
{
    const uint32_t timestamp_now = micros();

    if (!has_sample)
    {
        error_prev = error;
        output_prev = kp * error;
        if (output_limit != 0)
            output_prev = clamp(output_prev, -output_limit, output_limit);
        timestamp_prev = timestamp_now;
        has_sample = true;
        return output_prev;
    }

    const uint32_t elapsedUs = timestamp_now - timestamp_prev;
    if (elapsedUs == 0) // if the time hasn't advanced, return the previous output to avoid division by zero
        return output_prev;

    const float Ts = static_cast<float>(elapsedUs) * 1.0e-6f;

    float proportional = kp * error;
    float integral = integral_prev + ki * Ts * 0.5f * (error + error_prev);
    if (output_limit != 0)
        integral = clamp(integral, -output_limit, output_limit);
    float derivative = kd * (error - error_prev) / Ts;

    float output = proportional + integral + derivative;

    // limit the output and its rate of change
    if (output_limit != 0)
        output = clamp(output, -output_limit, output_limit);
    if (output_ramp_limit > 0)
    {
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp_limit)
            output = output_prev + output_ramp_limit * Ts;
        else if (output_rate < -output_ramp_limit)
            output = output_prev - output_ramp_limit * Ts;
    }
    if (output_limit != 0)
        output = clamp(output, -output_limit, output_limit);

    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;

    return output;
}

void PID_Controller::reset()
{
    error_prev = 0.0f;
    output_prev = 0.0f;
    integral_prev = 0.0f;
    timestamp_prev = 0;
    has_sample = false;
}
