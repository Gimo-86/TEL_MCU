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

    // SBUS 992 是中立
    auto norm = [](int value) {
        return map(value, 192, 1792, -255, 255);
    };

    int Vx = norm(ch2);         // 前後
    int Vy = norm(ch4);         // 左右
    int W  = norm(chRotate);    // 旋轉（現在用 0）

    // 麥克納姆四輪解算
    int m1 = Vx + Vy + W;
    int m2 = Vx - Vy - W;
    int m3 = Vx - Vy + W;
    int m4 = Vx + Vy - W;

    // 限幅
    m1 = constrain(m1, -255, 255);
    m2 = constrain(m2, -255, 255);
    m3 = constrain(m3, -255, 255);
    m4 = constrain(m4, -255, 255);

    // 輸出到馬達
    FL.setTargetRPM(m1);
    FR.setTargetRPM(m2);
    RL.setTargetRPM(m3);
    RR.setTargetRPM(m4);

    FL.updateControl();
    FR.updateControl();
    RL.updateControl();
    RR.updateControl();
}
