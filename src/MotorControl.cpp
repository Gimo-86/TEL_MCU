#include "MotorControl.h"
#include "PIDController.h"
#include "Config.h"

Motor::Motor(int pwmPin, int in1, int in2, int enca, int encb, float kp, float ki, float kd)
  : _pwmPin(pwmPin), _in1(in1), _in2(in2), _enca(enca), _encb(encb), _pid(kp, ki, kd),
    pulseCount(0), rotationDir(1), rpm(0), pwmValue(0),
    targetRPM(0), lastCount(0), lastTime(0) {}

void Motor::begin() {
  pinMode(_pwmPin, OUTPUT);
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_enca, INPUT_PULLUP);
  pinMode(_encb, INPUT_PULLUP);
  analogWrite(_pwmPin, 0);
}

void Motor::setTargetRPM(float target) {
  targetRPM = target;
}

void Motor::updateEncoder() {
  int stateA = digitalRead(_enca);
  int stateB = digitalRead(_encb);
  rotationDir = (stateA == stateB) ? 1 : -1;
  pulseCount += rotationDir;
}
void Motor::updateEncoderR() {
  int stateA = digitalRead(_enca);
  int stateB = digitalRead(_encb);
  rotationDir = (stateA == stateB) ? -1 : 1;
  pulseCount += rotationDir;
}

void Motor::updateControl() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  /* 更新脈波計數器 pulseCount f = 20Hz (T = 0.05s) */
  if (dt >= 0.05) {                 
    long countDiff = pulseCount - lastCount;
    lastCount = pulseCount;
    lastTime = now;

    /* 計算 RPM */
    float ppr = ENCODER_PPR;       // 由 Config.h 設定  
    float rawRPM = (countDiff / ppr) * (60.0 / dt);
    rpm = rawRPM * rotationDir;     // 讓 RPM 偵測值帶有方向

    /* PID 控制 */
    float output = _pid.compute(targetRPM, rpm, dt);
    pwmValue += (int)output;
    pwmValue = constrain(pwmValue, -255, 255);

    /* 限制 PWM 範圍 */
    if (pwmValue > 0) 
        pwmValue = constrain(pwmValue, minPWM, 255);
    else if (pwmValue < 0)
        pwmValue = constrain(pwmValue, -255, -minPWM);
    
    if (targetRPM == 0) {
        pwmValue = 0;
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }
    else {
      if (pwmValue > 0) {
        pwmValue = max(pwmValue, minPWM); // 正轉死區補償
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
      } 
      else if (pwmValue < 0) {
        pwmValue = min(pwmValue, -minPWM); // 反轉死區補償
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
      }
    }
    // ======實際輸出 PWM 取絕對值 ======
    analogWrite(_pwmPin, abs(pwmValue));
  }
}

float Motor::getRPM() { return rpm; }
int Motor::getPWM() { return pwmValue; }
float Motor::getTargetRPM() { return targetRPM; }