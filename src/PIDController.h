#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
  public:
    PIDController(float kp, float ki, float kd)
      : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0) {}

    // Compute PID output
    float compute(float target, float current, float dt) {
      float error = target - current;
      // Prevent integral windup
      if (integral < 1000 && integral > -1000) {
        integral += error * dt;
      }
      float derivative = (error - prevError) / dt;
      
      if (prevError == 0 && integral == 0) {
      derivative = 0; // 第一輪 derivative 不用算
    }

      if (dt < 0.001f) dt = 0.001f;  // avoid derivative explosion
      prevError = error;
      return Kp * error + Ki * integral + Kd * derivative;      
    }

    void reset() {
      integral = 0;
      prevError = 0;
    }

    float Kp, Ki, Kd;

  private:
    float prevError;
    float integral;
};

#endif