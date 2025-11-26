#include "FireMotorControl.h"
#include "Config.h"

void initFireMotors() {
    pinMode(FIRE_MOTOR1_PWM_PIN, OUTPUT);
    pinMode(FIRE_MOTOR2_PWM_PIN, OUTPUT);
    pinMode(FIRE_MOTOR1_DIR_PIN1, OUTPUT);
    pinMode(FIRE_MOTOR1_DIR_PIN2, OUTPUT);
    pinMode(FIRE_MOTOR2_DIR_PIN1, OUTPUT);
    pinMode(FIRE_MOTOR2_DIR_PIN2, OUTPUT);
    analogWrite(FIRE_MOTOR1_PWM_PIN, 0);
    analogWrite(FIRE_MOTOR2_PWM_PIN, 0);
    digitalWrite(FIRE_MOTOR1_DIR_PIN1, 1);
    digitalWrite(FIRE_MOTOR1_DIR_PIN2, 0);
    digitalWrite(FIRE_MOTOR2_DIR_PIN1, 1);
    digitalWrite(FIRE_MOTOR2_DIR_PIN2, 0);
}

void setFireMotors(bool on) {
    if (on) {
        analogWrite(FIRE_MOTOR1_PWM_PIN, FIRE_MOTOR_POWER);
        analogWrite(FIRE_MOTOR2_PWM_PIN, FIRE_MOTOR_POWER);
    } else {
        analogWrite(FIRE_MOTOR1_PWM_PIN, 0);
        analogWrite(FIRE_MOTOR2_PWM_PIN, 0);
    }
}
