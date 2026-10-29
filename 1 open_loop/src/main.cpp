#include <Arduino.h>

/**
 * Open loop FOC
 *  
 *  expected speed -> velocityOpenloop -> Uq(需求的力矩) -> Park -> Ualpha, Ubeta -> Clarke -> ua,ub,uc -> pwm output
 * 
 * 
 * DengFOC V4 Hardware.
 */

// PWM输出引脚定义
const int pwmA = 32;
const int pwmB = 33;
const int pwmC = 25;

const float voltage_power_supply = 12.6f;

float shaft_angle = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, duty_a = 0, duty_b = 0, duty_c = 0;

template <typename T>
static inline T clamp(T val, T low, T high)
{
    if (val < low)
        val = low;
    else if (val > high)
        val = high;

    return val;
};

// calculat electrical angle by mechanical angle and pole pairs
static inline float electrical_angle(float mechanical_angle, int pole_pairs)
{
    return (mechanical_angle * pole_pairs);
}

// normalize angle into [0, 2PI]
static float normalize_angle(float angle)
{
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

// 设置PWM到控制器输出
void setPwmOutput(float Ua, float Ub, float Uc)
{
    // 计算占空比
    // 限制占空比从0到1
    duty_a = clamp(Ua / voltage_power_supply, 0.0f, 1.0f);
    duty_b = clamp(Ub / voltage_power_supply, 0.0f, 1.0f);
    duty_c = clamp(Uc / voltage_power_supply, 0.0f, 1.0f);

    // 写入 PWM 占空比
    ledcWrite(0, duty_a * 255);
    ledcWrite(1, duty_b * 255);
    ledcWrite(2, duty_c * 255);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    angle_el = normalize_angle(angle_el + zero_electric_angle);
    // 帕克逆变换
    Ualpha = Ud * cos(angle_el) - Uq * sin(angle_el);
    Ubeta = Ud * sin(angle_el) + Uq * cos(angle_el);

    // 克拉克逆变换
    Ua = Ualpha + voltage_power_supply / 2;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
    setPwmOutput(Ua, Ub, Uc);
}

// 开环速度函数
float velocityOpenloop(float target_velocity)
{
    static uint32_t timestamp = 0;

    // 计算Loop的运行时间间隔
    uint32_t now_us = micros();
    uint32_t elapsed_us = now_us - timestamp;
    float Ts = elapsed_us * 1e-6f;

    // If the interval is zero or the loop stalled for over 0.5 seconds, use a small fallback.
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // 虚拟角度
    shaft_angle = normalize_angle(shaft_angle + target_velocity * Ts);
    float e_angle = electrical_angle(shaft_angle, 7);

    // 期望的力矩
    // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
    float Uq = voltage_power_supply / 3;

    setPhaseVoltage(Uq, 0, e_angle);

    timestamp = now_us; // 用于计算下一个时间间隔

    return Uq;
}

void align_to_zero_angle()
{
    float Ud = voltage_power_supply / 3;
    setPhaseVoltage(0, Ud, 0);
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

    align_to_zero_angle();
    delay(100);
    Serial.println("Rotor Alignment done.");

    delay(3000);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // velocityOpenloop(5);
}