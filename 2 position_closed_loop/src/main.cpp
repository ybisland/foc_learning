#include <Arduino.h>
#include "AS5600.h"

/**
 * Position closed-loop control
 * 
 *  Goal: keep the position at zero
 *  
 *  zero
 *   |
 *   |> position closed-loop controller -> Uq -> current open-loop output
 * 
 * DengFOC V4 Hardware.
 */

// PWM输出引脚
const int pwmA = 32;
const int pwmB = 33;
const int pwmC = 25;
// const int pwmA = 26;
// const int pwmB = 27;
// const int pwmC = 14;

const float voltage_power_supply = 12.6f;

#define _3PI_2 (4.71238898038f)
#define DIR (1)
#define POLE_PAIRS (7)

float shaft_angle = 0;
float zero_electric_angle = 0;

const float max_Uq = 6.0f, min_Uq = -6.0f;

template <typename T>
static inline T clamp(T val, T low, T high)
{
    if (val < low)
        val = low;
    else if (val > high)
        val = high;

    return val;
};

// normalize angle into [0, 2PI]
static float normalize_angle(float angle)
{
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

void setPhaseVoltage(float Uq, float Ud, float e_angle)
{
    // 归一化到 [0, 2pi]
    e_angle = normalize_angle(e_angle);

    // inverse park transform
    float Ualpha, Ubeta;
    Ualpha = Ud * cos(e_angle) - Uq * sin(e_angle);
    Ubeta = Ud * sin(e_angle) + Uq * cos(e_angle);

    // inverse clarke transform
    float Ua, Ub, Uc;
    Ua = Ualpha;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2;

    // svpwm
    float dc_bias = voltage_power_supply / 2;
    float duty_a, duty_b, duty_c;
    Ua += dc_bias, Ub += dc_bias, Uc += dc_bias;
    duty_a = clamp(Ua / voltage_power_supply, 0.0f, 1.0f);
    duty_b = clamp(Ub / voltage_power_supply, 0.0f, 1.0f);
    duty_c = clamp(Uc / voltage_power_supply, 0.0f, 1.0f);

    // output pwm
    ledcWrite(0, duty_a * 255);
    ledcWrite(1, duty_b * 255);
    ledcWrite(2, duty_c * 255);
}

void setup()
{
    Serial.begin(115200);

    // PWM设置，设置12引脚输出高电平，使能引脚
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH); // V4电机使能

    ledcSetup(0, 30000, 8); // pwm频道, 频率, 精度
    ledcSetup(1, 30000, 8); // pwm频道, 频率, 精度
    ledcSetup(2, 30000, 8); // pwm频道, 频率, 精度
    ledcAttachPin(pwmA, 0);
    ledcAttachPin(pwmB, 1);
    ledcAttachPin(pwmC, 2);
    Serial.println("Initialization done.");

    BeginSensor();
    {   // calibrate the encoder
        setPhaseVoltage(3, 0, _3PI_2);
        delay(3000);
        zero_electric_angle = normalize_angle((float)(DIR * POLE_PAIRS) * getAngle_Without_track() - zero_electric_angle);
        setPhaseVoltage(0, 0, _3PI_2);
    }

    delay(3000);
}


void keep_rotor_at_zero_position()
{
    const float Kp = max_Uq / (45.0f / 180.0f * PI); // 在 45° 时输出满幅输出
    const float target_angle = 0; // expected position is 0°
    float error, Uq, e_angle, m_angle;

    m_angle = getAngle_Without_track();
    e_angle = normalize_angle(DIR * POLE_PAIRS * m_angle - zero_electric_angle);

    error = DIR * (target_angle - getAngle());
    Uq = Kp * error;
    Uq = clamp(Uq, min_Uq, max_Uq);

    setPhaseVoltage(Uq, 0, e_angle);
    Serial.printf("ma %f ea %f e %f Uq %f\n", m_angle, e_angle, error, Uq);
}

void loop()
{
    // put your main code here, to run repeatedly:
    keep_rotor_at_zero_position();
}