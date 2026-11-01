#pragma once

#include <stdint.h>

/*
 * Usage:
 *   PID_Controller pid(kp, ki, kd, rampLimit, outputLimit);
 *   float output = pid.update(target - measured);
 *   pid.configure(newKp, newKi, newKd, newRampLimit, newOutputLimit);
 *   pid.reset(); // Clear controller history before starting again.
 *
 * ramp_limit limits output change per second; 0 disables it and a positive
 * value enables it.
 * output_limit bounds the output and integral; 0 disables it and a positive
 * value enables it.
 * The first update returns the P term, limited when outputLimit is nonzero.
 * A second update in the same micros() tick returns the previous output.
 */
class PID_Controller
{
public:
    PID_Controller();

    PID_Controller(float kp, float ki, float kd, float ramp_limit, float output_limit);

    // Updates parameters without resetting history; a nonzero output limit is
    // immediately applied to the stored output and integral.
    void configure(float kp, float ki, float kd, float ramp_limit, float output_limit);

    // The first call returns the limited P term without applying the output ramp.
    // A call in the same micros() tick returns the previous output.
    float update(float error);

    // Clears controller history; the next update uses the first-call behavior.
    void reset();

private:
    float kp;
    float ki;
    float kd;

    float output_ramp_limit; 
    float output_limit; 
    float error_prev;
    float output_prev;
    float integral_prev;
    uint32_t timestamp_prev;
    bool has_sample;
};
