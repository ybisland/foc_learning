#pragma once

/*
 * FOC_Controller usage:
 * 1. Create an instance and call configure() with the PWM pins and motor parameters.
 * 2. Configure the PID(s), then call calibrateSensor(); do not enable closed-loop
 *    control if calibration fails.
 * 3. Set a target, select a mode, and call run() repeatedly from loop().
 *
 * Example (velocity mode):
 *   FOC_Controller foc;
 *   void setup() {
 *       foc.configure(PIN_A, PIN_B, PIN_C, pole_pairs, 1, bus_voltage, Uq_limit);
 *       // foc.setVelocityFilterCutoff(10.0f); // optional, defaults to 10 Hz
 *       foc.setVelPID(kp, ki, kd, ramp_limit);
 *       if (foc.calibrateSensor()) {
 *           foc.setTargetVelocity(target_velocity);
 *           foc.setMode(FOC_MODE_VELOCITY_CLOSED_LOOP);
 *       }
 *   }
 *   void loop() { foc.run(); }
 *
 * For position mode, configure the position PID and use setTargetPosition() with
 * FOC_MODE_POSITION_CLOSED_LOOP instead.
 */

#include "LowPassFilter.h"
#include "AS5600.h"
#include "PID.h"

typedef enum {
    FOC_MODE_STOPPED = 0,
    FOC_MODE_POSITION_CLOSED_LOOP = 1,
    FOC_MODE_VELOCITY_CLOSED_LOOP = 2,
} FOCMode;

class FOC_Controller
{
public:
    FOC_Controller();

    // Call configure() before calibrateSensor() or enabling a closed-loop mode.
    // sensor_dir sets the positive sensor direction and must be 1 or -1.
    void configure(int pwm_pin_A, int pwm_pin_B, int pwm_pin_C,
                   int pole_pairs, int sensor_dir,
                   float bus_voltage, float Uq_limit);

    // Switching into a closed-loop mode resets its PID. Entering position mode
    // also removes previously accumulated full turns from the position feedback.
    void setMode(FOCMode mode);

    void setVelPID(float kp, float ki, float kd, float ramp_limit);

    void setPosPID(float kp, float ki, float kd, float ramp_limit);

    // Sets the velocity feedback low-pass cutoff in Hz; call after configure().
    // cutoff_hz must be positive and finite. Changing it does not reset the filter.
    void setVelocityFilterCutoff(float cutoff_hz);

    void setTargetVelocity(float target_velocity);

    void setTargetPosition(float target_position);

    float getActualVelocity() const;

    // Returns the turn-tracked angle with turns before the latest position-mode
    // entry excluded.
    double getActualPosition() const;

    // Call after configure(); returns false if the sensor cannot be read.
    bool calibrateSensor();

    // Runs the selected mode; sensor read failures produce zero phase voltage.
    void run();

private:
    static int controller_count;
    int controller_id;
    FOCMode mode;

    LowPassFilter vel_lpf;
    PID_Controller vel_pid, pos_pid;
    Sensor_AS5600 sensor;
    int pwm_pin_A, pwm_pin_B, pwm_pin_C;
    int pole_pairs, sensor_dir;

    int ledc_channel_A, ledc_channel_B, ledc_channel_C;

    float target_velocity, actual_velocity;
    double target_position, actual_position;
    double position_offset;

    float zero_electric_angle;

    float bus_voltage;
    float Uq_limit; // Maximum voltage to apply to the q-axis, in volts.

    float Uq, Ud, m_angle, e_angle, error; // Latest q- and d-axis voltages, in volts.

    void setPhaseVoltage(float Uq, float Ud, float angle_el);
    void runPositionClosedLoop();
    void runVelocityClosedLoop();
};