#include "ElevationControl.h"
#include "Config.h"

static void driveElevation(int speed) {
    // speed: 0..ELEVATION_POWER_LEVEL
    speed = constrain(speed, 0, 255);
    if (speed == 0) {
        analogWrite(ELEVATION_PWM_PIN, 0);
        digitalWrite(ELEVATION_DIR_PIN, LOW);
        return;
    }
    analogWrite(ELEVATION_PWM_PIN, speed);
}

void initElevation() {
    pinMode(ELEVATION_PWM_PIN, OUTPUT);
    pinMode(ELEVATION_DIR_PIN, OUTPUT);
    stopElevationMotor();
}

void stopElevationMotor() {
    analogWrite(ELEVATION_PWM_PIN, 0);
    digitalWrite(ELEVATION_DIR_PIN, LOW);
}

void setElevationFromChannel(uint16_t chValue) {
    // Use SBUS_NEUTRAL and ELEVATION_DEADZONE from Config.h
    if (chValue >= SBUS_NEUTRAL + ELEVATION_DEADZONE) {
        // proportionally scale speed from deadzone to max
        int speed = map((int)chValue, SBUS_NEUTRAL + ELEVATION_DEADZONE, 1792, 0, ELEVATION_POWER_LEVEL);
        speed = constrain(speed, 0, ELEVATION_POWER_LEVEL);
        // DIR HIGH increases angle
        digitalWrite(ELEVATION_DIR_PIN, LOW);
        driveElevation(speed);
    }
    else if (chValue <= SBUS_NEUTRAL - ELEVATION_DEADZONE) {
        int speed = map((int)chValue, 192, SBUS_NEUTRAL - ELEVATION_DEADZONE, ELEVATION_POWER_LEVEL, 0);
        speed = constrain(speed, 0, ELEVATION_POWER_LEVEL);
        // DIR LOW decreases angle
        digitalWrite(ELEVATION_DIR_PIN, HIGH);
        driveElevation(speed);
    }
    else {
        // within deadzone: stop
        stopElevationMotor();
    }
}
