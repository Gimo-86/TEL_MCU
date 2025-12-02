#include "ElevationControl.h"
#include "Config.h"

static void driveElevation(int speed) {
    // speed: 0..ELEVATION_POWER_LEVEL
    speed = constrain(speed, -ELEVATION_POWER_LEVEL, ELEVATION_POWER_LEVEL);
    if (speed == 0) {
        analogWrite(ELEVATION_R_PWM_PIN, 0);
        analogWrite(ELEVATION_L_PWM_PIN, 0);
        digitalWrite(ELEVATION_R_EN_PIN, LOW);
        digitalWrite(ELEVATION_L_EN_PIN, LOW);
        return;
    } else if (speed > 0) {
        analogWrite(ELEVATION_R_PWM_PIN, speed);
        analogWrite(ELEVATION_L_PWM_PIN, 0);
        digitalWrite(ELEVATION_R_EN_PIN, HIGH);
        digitalWrite(ELEVATION_L_EN_PIN, HIGH);
    } else {
        analogWrite(ELEVATION_R_PWM_PIN, 0);
        analogWrite(ELEVATION_L_PWM_PIN, -speed);
        digitalWrite(ELEVATION_R_EN_PIN, HIGH);
        digitalWrite(ELEVATION_L_EN_PIN, HIGH);
    }
}

void initElevation() {
    pinMode(ELEVATION_R_PWM_PIN, OUTPUT);
    pinMode(ELEVATION_L_PWM_PIN, OUTPUT);
    pinMode(ELEVATION_R_EN_PIN, OUTPUT);
    pinMode(ELEVATION_L_EN_PIN, OUTPUT);
    stopElevationMotor();
}

void stopElevationMotor() {
    analogWrite(ELEVATION_R_PWM_PIN, 0);
    analogWrite(ELEVATION_L_PWM_PIN, 0);
    digitalWrite(ELEVATION_R_EN_PIN, LOW);
    digitalWrite(ELEVATION_L_EN_PIN, LOW);
}

void setElevationFromChannel(uint16_t chValue) {
    if (chValue <= SBUS_NEUTRAL - ELEVATION_DEADZONE || chValue >= SBUS_NEUTRAL + ELEVATION_DEADZONE) {
        int speed = map((int)chValue, 192, 1792, -ELEVATION_POWER_LEVEL, ELEVATION_POWER_LEVEL);
        speed = constrain(speed, -ELEVATION_POWER_LEVEL, ELEVATION_POWER_LEVEL);
        driveElevation(speed);
    }
    else {
        stopElevationMotor();
    }
}
