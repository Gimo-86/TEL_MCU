#include "MotorControl.h"
#include "PIDController.h"
#include "Config.h"

// Global motor instances array for ISR access
Motor* motorInstances[4] = {nullptr, nullptr, nullptr, nullptr};
static uint8_t motorCount = 0;
static int dir = 1;
static int prevDir = 0;

// Static ISR handlers
void motorISR0() { if(motorInstances[0]) motorInstances[0]->updateEncoder(); }
void motorISR1() { if(motorInstances[1]) motorInstances[1]->updateEncoder(); }
void motorISR2() { if(motorInstances[2]) motorInstances[2]->updateEncoder(); }
void motorISR3() { if(motorInstances[3]) motorInstances[3]->updateEncoder(); }

static void (*isrHandlers[4])() = {motorISR0, motorISR1, motorISR2, motorISR3};

Motor::Motor(int pwmPin, int in1, int in2, int enca, float kp, float ki, float kd)
  : _pwmPin(pwmPin), _in1(in1), _in2(in2), _enca(enca), _pid(kp, ki, kd),
    pulseCount(0), rpm(0), pwmValue(0),
    targetRPM(0), lastCount(0), lastTime(0) {}

void Motor::begin() {
  pinMode(_pwmPin, OUTPUT);
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_enca, INPUT_PULLUP);
  analogWrite(_pwmPin, 0);
  
  // Register this motor instance for ISR access
  if (motorCount < 4) {
    motorInstances[motorCount] = this;
    attachInterrupt(digitalPinToInterrupt(_enca), isrHandlers[motorCount], CHANGE);
    motorCount++;
  }
}

void Motor::setTargetRPM(float target) {
  this->targetRPM = target;
}

//Called from ISR - Single phase encoder
void Motor::updateEncoder() {
  pulseCount++;
  // Direction is later determined in updateControl based on targetRPM sign.
}


void Motor::updateControl() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  /* 更新脈波計數器 pulseCount f = 20Hz (T = 0.05s) */
  if (dt >= 0.05) {                 
    long countDiff = pulseCount - lastCount;
    lastCount = pulseCount;
    lastTime = now;

    /* 計算 RPM - single phase encoder only counts up */
    float ppr = ENCODER_PPR;                       // 由 Config.h 設定  
    float rawRPM = (countDiff / ppr) * (60.0 / dt);
    
    /* Apply direction from targetRPM sign */
    if (targetRPM < 0) {
      rpm = -rawRPM;  // Negative RPM when motor commanded reverse
    } else {
      rpm = rawRPM;   // Positive RPM when motor commanded forward
    }

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

//Functions for debugging
float Motor::getRPM() { return rpm; }
int Motor::getPWM() { return pwmValue; }
float Motor::getTargetRPM() { return targetRPM; }