#include "FOC.h"
#include "serial_command.h"
#include <Arduino.h>

/**
 * DengFOC V4 hardware serial control for the FOC controller.
 */

#define BUS_VOLTAGE (12.6f)
#define DIR (1)
#define POLE_PAIRS (7)

#define M0_PIN_A (32)
#define M0_PIN_B (33)
#define M0_PIN_C (25)

FOC_Controller m0_foc;

void setup()
{
    Serial.begin(115200);

    // DengFOC V4 board motor enable
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

    m0_foc.configure(
        M0_PIN_A, M0_PIN_B, M0_PIN_C,
        POLE_PAIRS, DIR,
        BUS_VOLTAGE, 6.0f);
    m0_foc.setVelocityFilterCutoff(1.0f);

    m0_foc.calibrateSensor();
    m0_foc.setMode(FOC_MODE_STOPPED);

    // the range of Uq is -6V~6V, the ramp_limit is set to 5V/s to avoid sudden changes in voltage
    m0_foc.setVelPID(1.0f, 0.8f, 0.0f, 6.0f);
    m0_foc.setPosPID(1.0f, 0.8f, 0.0f, 6.0f);

    serial_command_init(&m0_foc);
}

void loop()
{
    receive_serial_commands();
    m0_foc.run();
}
