#include "MecanumControl.h"
#include "MotorControl.h"
#include "Config.h"
#include <Arduino.h>


MotorSystem::MotorSystem()
    : FL(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, MOTOR1_DIR1_PIN, FL_ENA, PID_FL_KP, PID_FL_KI, PID_FL_KD),
      FR(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, MOTOR2_DIR1_PIN, FR_ENA, PID_FR_KP, PID_FR_KI, PID_FR_KD),
      RL(MOTOR3_PWM_PIN, MOTOR3_DIR_PIN, MOTOR3_DIR1_PIN, RL_ENA, PID_RL_KP, PID_RL_KI, PID_RL_KD),
      RR(MOTOR4_PWM_PIN, MOTOR4_DIR_PIN, MOTOR4_DIR1_PIN, RR_ENA, PID_RR_KP, PID_RR_KI, PID_RR_KD)
{}

 void MotorSystem::initMecanum() {
    FL.begin();
    FR.begin();
    RL.begin();
    RR.begin();
 }
 
/*
    Channel mapping used by mecanumDrive:
    - CH2 (index 1) -> 前後（正前進 / 負後退）
    - CH4 (index 3) -> 左右平移（正右 / 負左）
    - CH1 (index 0) -> 旋轉（左轉 / 右轉）
*/

void MotorSystem::mecanumDrive(int ch2, int ch4, int chRotate) {
    //Serial.println("MECA");
    // SBUS 992 是中立
    auto norm = [](int value) -> float {
        // map SBUS (approx 192~1792) to -CONFIG_MAX_RPM .. +CONFIG_MAX_RPM
        const float inMin = 192.0f;
        const float inMax = 1792.0f;
        float t = (float(value) - inMin) / (inMax - inMin);
        return (t * 2.0f - 1.0f) * CONFIG_MAX_RPM;
    };

    float Vx = norm(ch2);         // 前後 (RPM)
    float Vy = norm(ch4);         // 左右 (RPM)
    float W  = norm(chRotate);    // 旋轉 (RPM)

    Serial.println(Vx, 4);
    Serial.println(Vy, 4);
    Serial.println(W, 4);

    // 麥克納姆四輪解算
    float m1 = (Vx + Vy + W) * kFL;  // FL
    float m2 = (Vx - Vy - W) * kFR;  // FR
    float m3 = (Vx - Vy + W) * kRL;  // RL (無 encoder → 補償用)
    float m4 = (Vx + Vy - W) * kRR;  // RR

    // Apply per-wheel direction multipliers (in case motor wiring/polarity differs)
    m1 *= MOTOR_FL_DIR;
    m2 *= MOTOR_FR_DIR;
    m3 *= MOTOR_RL_DIR;
    m4 *= MOTOR_RR_DIR;

    // 直接以 RPM 為目標值輸出到馬達 PID
    setTargetRPMs(m1, m2, m3, m4);
}

void MotorSystem::setTargetRPMs(float m1, float m2, float m3, float m4) {
    m1 = constrain(m1, -CONFIG_MAX_RPM, CONFIG_MAX_RPM);
    m2 = constrain(m2, -CONFIG_MAX_RPM, CONFIG_MAX_RPM);
    m3 = constrain(m3, -CONFIG_MAX_RPM, CONFIG_MAX_RPM);
    m4 = constrain(m4, -CONFIG_MAX_RPM, CONFIG_MAX_RPM);

    FL.setTargetRPM(m1);
    FR.setTargetRPM(m2);
    RL.setTargetRPM(m3);
    RR.setTargetRPM(m4);

    FL.updateControl();
    FR.updateControl();
    RL.updateControl();
    RR.updateControl();
}

void MotorSystem::stop() {
    FL.setTargetRPM(0);
    FR.setTargetRPM(0);
    RL.setTargetRPM(0);
    RR.setTargetRPM(0);

    FL.updateControl();
    FR.updateControl();
    RL.updateControl();
    RR.updateControl();
}
