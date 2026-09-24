#pragma once
#include <SimpleFOC.h>

// ---------------------------------------------------------------------------
// Analog-hall shaft angle with a 2-state constant-velocity Kalman filter.
// Same structure as OrbitDrive3.0 MT6835EKF, but fed by GenericSensor (halls).
//
// State  x = [theta; omega]  (rad, rad/s) on the continuous getAngle() path.
// Velocity returned to SimpleFOC is the filtered omega (quiet vs finite-diff).
// ---------------------------------------------------------------------------
class HallAngleEKF : public GenericSensor {
 public:
  HallAngleEKF(float (*readCallback)() = nullptr, void (*initCallback)() = nullptr)
      : GenericSensor(readCallback, initCallback) {}

  bool  ekf_enabled = true;
  float q_angle = 0.001f;   // process noise, angle
  float q_vel   = 40.0f;    // process noise, velocity (lower = smoother)
  float r_meas  = 0.008f;   // measurement noise (rad^2) — halls noisier than MT6835

  void update() override {
    Sensor::update();
    if (!ekf_enabled) return;

    float z = getAngle();
    long now = _micros();
    float dt = (now - _ekf_ts) * 1e-6f;
    _ekf_ts = now;

    if (!_ekf_init || dt <= 0.0f || dt > 0.5f) {
      _theta = z;
      _omega = 0.0f;
      _P00 = 1.0f;
      _P01 = 0.0f;
      _P10 = 0.0f;
      _P11 = 1.0f;
      _ekf_init = true;
      return;
    }

    // Predict
    _theta += _omega * dt;
    float P00 = _P00 + dt * (_P10 + _P01) + dt * dt * _P11 + q_angle * dt;
    float P01 = _P01 + dt * _P11;
    float P10 = _P10 + dt * _P11;
    float P11 = _P11 + q_vel * dt;

    // Update
    float y = z - _theta;
    float S = P00 + r_meas;
    float K0 = P00 / S;
    float K1 = P10 / S;
    _theta += K0 * y;
    _omega += K1 * y;

    _P00 = (1.0f - K0) * P00;
    _P01 = (1.0f - K0) * P01;
    _P10 = P10 - K1 * P00;
    _P11 = P11 - K1 * P01;
  }

  float getVelocity() override {
    if (!ekf_enabled) return Sensor::getVelocity();
    return _omega;
  }

  float getFilteredAngle() const { return _theta; }

  void resetEkf() { _ekf_init = false; }

 private:
  float _theta = 0.0f, _omega = 0.0f;
  float _P00 = 1.0f, _P01 = 0.0f, _P10 = 0.0f, _P11 = 1.0f;
  long _ekf_ts = 0;
  bool _ekf_init = false;
};
