#include "FOC.h"
#include <Arduino.h>

int FOC_Controller::controller_count = 0;

#define _3PI_2 (4.71238898038f)

// normalize angle into [0, 2PI]
static float normalize_angle(float angle)
{
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

template <typename T>
static inline T clamp(T val, T low, T high)
{
    if (val < low)
        val = low;
    else if (val > high)
        val = high;

    return val;
};

void FOC_Controller::configure(int pwm_pin_A, int pwm_pin_B, int pwm_pin_C, 
    int pole_pairs, int sensor_dir, 
    float bus_voltage, float Uq_limit)
{
    this->pwm_pin_A = pwm_pin_A;
    this->pwm_pin_B = pwm_pin_B;
    this->pwm_pin_C = pwm_pin_C;
    this->pole_pairs = pole_pairs;
    this->sensor_dir = sensor_dir;
    this->bus_voltage = bus_voltage;
    this->target_velocity = 0;
    this->target_position = 0;
    this->actual_velocity = 0;
    this->actual_position = 0;
    this->position_offset = 0;
    this->zero_electric_angle = 0.0f;
    this->Uq_limit = Uq_limit;

    this->mode = FOC_MODE_STOPPED;
    vel_pid.reset();
    pos_pid.reset();
    vel_lpf.reset();

    ledc_channel_A = controller_id * 3 + 0;
    ledc_channel_B = controller_id * 3 + 1;
    ledc_channel_C = controller_id * 3 + 2;

    ledcSetup(ledc_channel_A, 30000, 8); // pwm频道, 频率, 精度
    ledcSetup(ledc_channel_B, 30000, 8); // pwm频道, 频率, 精度
    ledcSetup(ledc_channel_C, 30000, 8); // pwm频道, 频率, 精度
    ledcAttachPin(pwm_pin_A, ledc_channel_A);
    ledcAttachPin(pwm_pin_B, ledc_channel_B);
    ledcAttachPin(pwm_pin_C, ledc_channel_C);

    AS5600Status sensorStatus = sensor.begin(Wire, 19, 18, 400000UL);
    if (sensorStatus != AS5600Status::Ok) {
        Serial.printf("AS5600 initialization failed: %u\n",
                      static_cast<unsigned>(sensorStatus));
        return;
    }

    // vel_pid = PID_Controller{.P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = bus_voltage / 2};
    vel_pid.configure(0, 0, 0, 100000, bus_voltage / 2);
    pos_pid.configure(0, 0, 0, 100000, bus_voltage / 2);

    vel_lpf.configure(10.0f);
}

void FOC_Controller::setMode(FOCMode mode)
{
    if (this->mode != mode)
    {
        if (mode == FOC_MODE_POSITION_CLOSED_LOOP)
        {
            pos_pid.reset();
            position_offset = sensor.getAngle() - sensor.getMechanicalAngle();
            actual_position = sensor.getAngle() - position_offset;
        }
        else if (mode == FOC_MODE_VELOCITY_CLOSED_LOOP)
            vel_pid.reset();
    }

    this->mode = mode;
    setPhaseVoltage(0, 0, 0);
}

void FOC_Controller::setPhaseVoltage(float Uq, float Ud, float e_angle)
{
    // 归一化到 [0, 2pi]
    e_angle = normalize_angle(e_angle);

    // inverse park transform: Ud, Uq -> Ualpha, Ubeta
    float Ualpha, Ubeta;
    Ualpha = Ud * cos(e_angle) - Uq * sin(e_angle);
    Ubeta = Ud * sin(e_angle) + Uq * cos(e_angle);

    // inverse clarke transform: Ualpha, Ubeta -> Ua, Ub, Uc
    float Ua, Ub, Uc;
    Ua = Ualpha;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2;

    // spwm
    float dc_bias = bus_voltage / 2;
    float duty_a, duty_b, duty_c;
    Ua += dc_bias, Ub += dc_bias, Uc += dc_bias;
    duty_a = clamp(Ua / bus_voltage, 0.0f, 1.0f);
    duty_b = clamp(Ub / bus_voltage, 0.0f, 1.0f);
    duty_c = clamp(Uc / bus_voltage, 0.0f, 1.0f);

    // output pwm
    ledcWrite(ledc_channel_A, duty_a * 255);
    ledcWrite(ledc_channel_B, duty_b * 255);
    ledcWrite(ledc_channel_C, duty_c * 255);
}

void FOC_Controller::setTargetVelocity(float target_velocity)
{
    this->target_velocity = target_velocity;
}

void FOC_Controller::setTargetPosition(float target_position)
{
    this->target_position = target_position;
}

void FOC_Controller::setVelPID(float kp, float ki, float kd, float ramp_limit)
{
    vel_pid.configure(kp, ki, kd, ramp_limit, bus_voltage / 2);
}

void FOC_Controller::setPosPID(float kp, float ki, float kd, float ramp_limit)
{
    pos_pid.configure(kp, ki, kd, ramp_limit, bus_voltage / 2);
}

void FOC_Controller::setVelocityFilterCutoff(float cutoff_hz)
{
    vel_lpf.configure(cutoff_hz);
}

float FOC_Controller::getActualVelocity() const
{
    return actual_velocity;
}

double FOC_Controller::getActualPosition() const
{
    return actual_position;
}

void FOC_Controller::run()
{
    // read the sensor data even if the foc is stopped
    if (sensor.update() != AS5600Status::Ok) {
        setPhaseVoltage(0, 0, 0);
        return;
    }

    actual_position = sensor.getAngle() - position_offset;
    actual_velocity = vel_lpf.update(sensor_dir * sensor.getVelocity());

    if (mode == FOC_MODE_POSITION_CLOSED_LOOP)
        runPositionClosedLoop();
    else if (mode == FOC_MODE_VELOCITY_CLOSED_LOOP)
        runVelocityClosedLoop();

    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime >= 200) {
        lastPrintTime = millis();
        Serial.printf("chennel 0: %f,%f,%f\n",
                      actual_position,
                      actual_velocity,
                      Uq);
    }
}

void FOC_Controller::runPositionClosedLoop()
{
    double error;
    float Uq, Ud, e_angle, m_angle;

    m_angle = sensor.getMechanicalAngle();
    e_angle = normalize_angle(sensor_dir * pole_pairs * m_angle - zero_electric_angle);

    error = sensor_dir * (target_position - actual_position);

    Uq = pos_pid.update(error);
    Uq = clamp(Uq, -1 * Uq_limit, Uq_limit);
    Ud = 0;

    setPhaseVoltage(Uq, Ud, e_angle);
}

void FOC_Controller::runVelocityClosedLoop()
{
    m_angle = sensor.getMechanicalAngle();
    e_angle = normalize_angle(sensor_dir * pole_pairs * m_angle - zero_electric_angle);

    error = target_velocity - actual_velocity;

    Uq = vel_pid.update(error);
    Uq = clamp(Uq, -1 * Uq_limit, Uq_limit);
    Ud = 0;

    setPhaseVoltage(Uq, Ud, e_angle);
}

FOC_Controller::FOC_Controller()
{
    controller_id = controller_count++;
    zero_electric_angle = 0.0f;
}

bool FOC_Controller::calibrateSensor()
{
    // Align the sensor to the motor's electrical angle.
    setPhaseVoltage(3, 0, _3PI_2);
    delay(1000);
    if (sensor.update() != AS5600Status::Ok) {
        setPhaseVoltage(0, 0, _3PI_2);
        Serial.println("AS5600 update failed during alignment.");
        return false;
    }
    zero_electric_angle = normalize_angle(
        (float)(sensor_dir * pole_pairs) * sensor.getMechanicalAngle());
    setPhaseVoltage(0, 0, _3PI_2);
    return true;
}