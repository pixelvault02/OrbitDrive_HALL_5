/**
 * OrbitDrive_HALL_5 - Pitch/tilt FOC (soft -135/+135 deg, CAN node 1)
 *   MCU     : STM32G431CBU6
 *   Driver  : STSPIN233 (3PWM + common enable)
 *   Sensor  : Analog halls (Clarke -> atan2 -> GenericSensor)
 *   Comms   : CAN (FDCAN1 / TJA1051) + USART1
 *   Storage : FlashStorage_STM32 (one-time FOC electrical calib + tuning)
 *
 * Boot sequence:
 *   1. Init hall + GenericSensor + driver
 *   2. Load EEPROM electrical calib if valid (skip align); else FOC align once + save
 *   3. initFOC
 *   4. bootHomeCwCcwMid() - hard stops -> zero at CCW+45 (soft -45/+90)
 *   5. Angle hold at 0 with soft limits
 *   6. CAN + serial ready
 */

#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <SimpleFOC.h>
#include <FlashStorage_STM32.h>
#include "STM32_CAN.h"
#include "board_pins.h"
#include "orbit_protocol.h"
#include "analog_hall.h"
#include "hall_angle_ekf.h"

// ---------------------------------------------------------------------------
// Motor / supply — same limits as OrbitDrive3.0 (phase R ~11 Ω, 7 pp)
// ---------------------------------------------------------------------------
#ifndef MOTOR_POLE_PAIRS
#define MOTOR_POLE_PAIRS 7
#endif
#ifndef POWER_SUPPLY_VOLTAGE
#define POWER_SUPPLY_VOLTAGE 9.0f
#endif
#ifndef DRIVER_VOLTAGE_LIMIT
#define DRIVER_VOLTAGE_LIMIT 9.0f
#endif
#ifndef MOTOR_VOLTAGE_LIMIT
#define MOTOR_VOLTAGE_LIMIT 4.0f
#endif
#ifndef ALIGN_VOLTAGE
#define ALIGN_VOLTAGE 1.5f
#endif
#ifndef MOTOR_PHASE_RESISTANCE
#define MOTOR_PHASE_RESISTANCE 11.2f
#endif
#ifndef MOTOR_KV
#define MOTOR_KV 200.0f             // hall motor KV (3.0 used 140 on encoder build)
#endif
#ifndef MAX_CURRENT_A
#define MAX_CURRENT_A 0.50f          // normal angle hold / osc / home seek
#endif
#ifndef TONE_VOLTAGE
#define TONE_VOLTAGE 6.0f
#endif

// Home holds a slow shaft speed with a 50 Hz current trim. The fast
// velocity PID is not used — hall speed noise is what made homing buzz.
#define BOOT_AUTO_HOME       1
#define HOME_CRUISE_DPS      80.0f   // seek and descent
#define HOME_CREEP_DPS       40.0f   // back off a stop
#define HOME_REVERSE_ACCEL   60.0f   // deg/s^2 leaving an end stop
#define HOME_ARREST_MS       200UL   // motion stopped → leave at once
#define HOME_IQ_SLEW         2.0f    // A/s — reverse to brake a fall
#define HOME_SEEK_RPM        8.0f    // sign only (direction)
#define HOME_TIMEOUT_MS      22000UL
#define HOME_MIN_TRAVEL_DEG  15.0f
#define HOME_NO_MOVE_MS      1800UL
#define HOME_UNSTICK_DEG     5.0f
#define HOME_END_OFFSET_DEG  8.0f
#define HOME_CLEAR_FRAC      0.08f
#define HOME_MID_MS          10000UL
#define HOME_MID_OK_DEG      5.0f
#define HOME_SOFT_IN_DEG     3.0f
// Hard travel is about 270° end to end, not an exact angle.
#define HOME_SPAN_MIN_DEG    200.0f
#define HOME_SPAN_MAX_DEG    340.0f
#define TRAVEL_MARGIN_DEG    0.5f
#define SOFT_TRAVEL_DEG      90.0f   // fallback until home (pitch CW side)
// End-zone protect (does not change PID gains)
#define END_ZONE_DEG         6.0f
#define END_SLEW_SCALE       0.35f
#define FOLDBACK_CURRENT_A   0.08f
#define JAM_ERR_DEG          12.0f
#define JAM_VEL_RAD_S        0.12f
#define JAM_UQ_ABS           0.85f
#define JAM_HOLD_MS          1500UL
#define JAM_GRACE_MS         12000UL

static const float DEG2RAD = PI / 180.0f;
static const float RAD2DEG = 180.0f / PI;

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
HallAngleEKF sensor = HallAngleEKF(readHallShaftAngle);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_DRV_EN);

STM32_CAN Can1(PIN_CAN_RX, PIN_CAN_TX);
static bool can_ready = false;

static uint8_t  can_node = ORBIT_DEF_CAN_NODE;
static uint16_t can_cmd_base = ORBIT_CAN_CMD_BASE;
static uint16_t can_id_telem = ORBIT_CAN_RPT_BASE + ORBIT_RPT_TELEMETRY;
static uint16_t can_id_param = ORBIT_CAN_RPT_BASE + ORBIT_RPT_PARAM;
static uint16_t can_id_halls = ORBIT_CAN_RPT_BASE + ORBIT_RPT_HALLS;
static uint16_t can_id_status = ORBIT_CAN_RPT_BASE + ORBIT_RPT_STATUS;
static uint16_t can_id_limits = ORBIT_CAN_RPT_BASE + ORBIT_RPT_LIMITS;
static uint16_t can_id_node = ORBIT_CAN_RPT_BASE + ORBIT_RPT_NODE;

static void applyCanNode(uint8_t node) {
  can_node = (node > ORBIT_CAN_NODE_MAX) ? ORBIT_CAN_NODE_MAX : node;
  can_cmd_base = ORBIT_CAN_CMD_FILTER(can_node);
  can_id_telem = ORBIT_CAN_RPT_ID(can_node, ORBIT_RPT_TELEMETRY);
  can_id_param = ORBIT_CAN_RPT_ID(can_node, ORBIT_RPT_PARAM);
  can_id_halls = ORBIT_CAN_RPT_ID(can_node, ORBIT_RPT_HALLS);
  can_id_status = ORBIT_CAN_RPT_ID(can_node, ORBIT_RPT_STATUS);
  can_id_limits = ORBIT_CAN_RPT_ID(can_node, ORBIT_RPT_LIMITS);
  can_id_node = ORBIT_CAN_RPT_ID(can_node, ORBIT_RPT_NODE);
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
OrbitConfig cfg;
float mech_zero_offset = 0.0f;
float cmd_angle_deg = 0.0f;
float target_position = 0.0f;   // FOC angle setpoint (profiled)
static float angle_goal = 0.0f; // commanded goal (absolute shaft rad)
static float profile_vel = 0.0f; // rad/s command-profile velocity

static bool homed = false;
static bool soft_limits_valid = false;  // soft window usable (user + optional hard clamp)
static float travel_min_deg = ORBIT_ANGLE_MIN_DEG;
static float travel_max_deg = ORBIT_ANGLE_MAX_DEG;
static float hard_min_deg = ORBIT_ANGLE_MIN_DEG;
static float hard_max_deg = ORBIT_ANGLE_MAX_DEG;
static bool hard_limits_valid = false;
static bool home_move = false;
static float home_track_deg = 0.0f;
static float home_cmd_track = 0.0f;
static float home_v_cmd = 0.0f;
static float home_zero_track = 0.0f;
static float home_track_prev = 0.0f;
static bool vel_mode = false;
static float target_vel_mech = 0.0f;  // rad/s when vel_mode
static bool endstop_fault = false;    // latched; cleared by SET_ENABLE 1
static uint32_t jam_sense_t0 = 0;
static uint32_t jam_grace_until = 0;   // millis deadline — skip jam trip

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
static void setStatusLed(bool on) { digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW); }

static void blinkStatus(uint8_t times, uint16_t on_ms = 120, uint16_t off_ms = 120) {
  for (uint8_t i = 0; i < times; i++) {
    setStatusLed(true);  delay(on_ms);
    setStatusLed(false); delay(off_ms);
  }
}

// ---------------------------------------------------------------------------
// Startup tone
// ---------------------------------------------------------------------------
static void playTone(float freq, uint16_t duration_ms) {
  if (freq <= 1.0f) { motor.setPhaseVoltage(0, 0, 0); delay(duration_ms); return; }
  float el = motor.electricalAngle();
  uint32_t half_us = (uint32_t)(500000.0f / freq);
  uint32_t cycles = ((uint32_t)duration_ms * 1000UL) / (2UL * half_us);
  for (uint32_t i = 0; i < cycles; i++) {
    motor.setPhaseVoltage(0,  TONE_VOLTAGE, el);
    delayMicroseconds(half_us);
    motor.setPhaseVoltage(0, -TONE_VOLTAGE, el);
    delayMicroseconds(half_us);
  }
  motor.setPhaseVoltage(0, 0, el);
}

static void startupChime() {
  playTone(659.25f,  90);
  playTone(783.99f,  90);
  playTone(1046.50f, 90);
  playTone(1318.51f, 110);
  playTone(1567.98f, 150);
  playTone(1318.51f, 90);
  playTone(1046.50f, 240);
  motor.setPhaseVoltage(0, 0, 0);
}

// ---------------------------------------------------------------------------
// Angle helpers + soft limits
// ---------------------------------------------------------------------------
static float wrapPI(float a) {
  while (a >  PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

static float softMinDeg() {
  if (soft_limits_valid) return travel_min_deg + TRAVEL_MARGIN_DEG;
  return ORBIT_ANGLE_MIN_DEG;
}

static float softMaxDeg() {
  if (soft_limits_valid) return travel_max_deg - TRAVEL_MARGIN_DEG;
  return ORBIT_ANGLE_MAX_DEG;
}

/** Normalize cfg soft window and push into travel_* (clamped to hard span if known). */
static void applySoftWindow() {
  if (isnan(cfg.soft_min_deg) || isinf(cfg.soft_min_deg) ||
      isnan(cfg.soft_max_deg) || isinf(cfg.soft_max_deg) ||
      cfg.soft_max_deg < cfg.soft_min_deg + 4.0f) {
    cfg.soft_min_deg = ORBIT_DEF_SOFT_MIN;
    cfg.soft_max_deg = ORBIT_DEF_SOFT_MAX;
  }
  // Absolute protocol range for pitch UI editing
  if (cfg.soft_min_deg < -160.0f) cfg.soft_min_deg = -160.0f;
  if (cfg.soft_max_deg > 160.0f) cfg.soft_max_deg = 160.0f;
  if (cfg.soft_min_deg > -2.0f) cfg.soft_min_deg = -2.0f;
  if (cfg.soft_max_deg < 2.0f) cfg.soft_max_deg = 2.0f;

  float lo = cfg.soft_min_deg;
  float hi = cfg.soft_max_deg;
  float hard_span = hard_max_deg - hard_min_deg;
  if (hard_limits_valid && hard_span > 200.0f) {
    if (lo < hard_min_deg) lo = hard_min_deg;
    if (hi > hard_max_deg) hi = hard_max_deg;
    if (hi < lo + 4.0f) {
      lo = hard_min_deg;
      hi = hard_max_deg;
    }
  }
  travel_min_deg = lo;
  travel_max_deg = hi;
  soft_limits_valid = true;
}

/**
 * Zero is the midpoint, so each hard stop sits about half the span from center.
 * Tracks are continuous degrees from the start of this home, so a hall
 * wrap cannot swap the two ends.
 * Returns false if the measured span is outside 200–340° (around 270°).
 */
static bool adoptPitchFromTracks(float ccw_track, float cw_track) {
  float span = cw_track - ccw_track;
  float span_abs = fabsf(span);
  if (span_abs < HOME_SPAN_MIN_DEG || span_abs > HOME_SPAN_MAX_DEG) {
    Serial.print(F("  span reject "));
    Serial.println(span_abs, 1);
    return false;
  }
  float zero_track = 0.5f * (ccw_track + cw_track);
  home_zero_track = zero_track;
  float t_lo = fminf(ccw_track, cw_track);
  float t_hi = fmaxf(ccw_track, cw_track);
  if (zero_track <= t_lo + 5.0f || zero_track >= t_hi - 5.0f) return false;

  motor.loopFOC();
  float delta = zero_track - home_track_deg;
  mech_zero_offset = motor.shaft_angle + delta * DEG2RAD;

  float ang_ccw = ccw_track - zero_track;  // about -135
  float ang_cw = cw_track - zero_track;    // about +135
  float ang_lo = fminf(ang_ccw, ang_cw);
  float ang_hi = fmaxf(ang_ccw, ang_cw);
  float clear = fmaxf(HOME_END_OFFSET_DEG, HOME_CLEAR_FRAC * span_abs);
  if (clear > span_abs * 0.15f) clear = span_abs * 0.15f;
  hard_min_deg = ang_lo + clear;
  hard_max_deg = ang_hi - clear;
  if (hard_max_deg < hard_min_deg + 10.0f) {
    hard_min_deg = ang_lo + 1.0f;
    hard_max_deg = ang_hi - 1.0f;
  }
  hard_limits_valid = true;
  cfg.soft_min_deg = ORBIT_DEF_SOFT_MIN;
  cfg.soft_max_deg = ORBIT_DEF_SOFT_MAX;
  applySoftWindow();

  Serial.print(F("  zero at mid  span="));
  Serial.print(span_abs, 1);
  Serial.print(F("  stops["));
  Serial.print(ang_ccw, 1);
  Serial.print(F(","));
  Serial.print(ang_cw, 1);
  Serial.println(F("]"));
  return true;
}

static float desiredDeg() { return wrapPI(target_position - mech_zero_offset) * RAD2DEG; }
static float actualDeg()  { return wrapPI(motor.shaft_angle - mech_zero_offset) * RAD2DEG; }

static void clearPidIntegrals() {
  motor.PID_velocity.reset();
  motor.PID_current_q.reset();
  motor.PID_current_d.reset();
}

static void setMotorEnable(bool on) {
  if (on) {
    endstop_fault = false;
    jam_sense_t0 = 0;
    motor.enable();
    motor.current_limit = MAX_CURRENT_A;
    motor.PID_velocity.limit = motor.current_limit;
    clearPidIntegrals();
    // Hold wherever we are to avoid a jump on re-enable
    sensor.update();
    target_position = motor.shaft_angle;
    angle_goal = motor.shaft_angle;
    cmd_angle_deg = actualDeg();
    profile_vel = 0.0f;
    vel_mode = false;
    motor.controller = MotionControlType::angle;
    Serial.println(F("MOTOR enable"));
  } else {
    clearPidIntegrals();
    motor.disable();
    Serial.println(F("MOTOR disable"));
  }
}

static void tripEndstopFault(const char *why) {
  if (endstop_fault) return;
  endstop_fault = true;
  Serial.print(F("ENDSTOP FAULT: "));
  Serial.println(why);
  // Hold command at actual, then disable drive
  sensor.update();
  float a = actualDeg();
  cmd_angle_deg = a;
  angle_goal = motor.shaft_angle;
  target_position = motor.shaft_angle;
  profile_vel = 0.0f;
  setMotorEnable(false);
}

static void applyAngleCommand(float deg) {
  if (endstop_fault || !motor.enabled) return;
  vel_mode = false;
  motor.controller = MotionControlType::angle;
  cmd_angle_deg = constrain(deg, softMinDeg(), softMaxDeg());
  float raw = mech_zero_offset + cmd_angle_deg * DEG2RAD;
  angle_goal = motor.shaft_angle + wrapPI(raw - motor.shaft_angle);
  // Slew <= 0 → snap immediately
  if (cfg.slew_rate <= 0.05f) {
    target_position = angle_goal;
    profile_vel = 0.0f;
  }
}

/**
 * Trapezoid angle profile: accelerate to slew_rate, cruise, decelerate into goal.
 * accel_rate/decel_rate in deg/s^2; 0 accel → legacy constant-slew steps.
 */
static void stepAngleSlew(float dt) {
  if (dt < 1e-5f) dt = 1e-5f;
  if (dt > 0.05f) dt = 0.05f;

  float err = wrapPI(angle_goal - target_position);
  if (cfg.slew_rate <= 0.05f) {
    target_position = angle_goal;
    profile_vel = 0.0f;
    return;
  }

  float vmax = cfg.slew_rate * DEG2RAD;
  float act = actualDeg();
  float lo = softMinDeg();
  float hi = softMaxDeg();
  if (err > 0.0f && (hi - act) < END_ZONE_DEG) {
    vmax *= END_SLEW_SCALE;
  } else if (err < 0.0f && (act - lo) < END_ZONE_DEG) {
    vmax *= END_SLEW_SCALE;
  }

  float a_acc = cfg.accel_rate * DEG2RAD;
  float a_dec = cfg.decel_rate * DEG2RAD;
  if (a_dec < 1e-6f) a_dec = a_acc;

  // Legacy path: no accel configured → constant slew (old behavior)
  if (a_acc < 1e-6f) {
    float max_step = vmax * dt;
    if (fabsf(err) <= max_step) {
      target_position = angle_goal;
      profile_vel = 0.0f;
    } else {
      float step = (err > 0.0f) ? max_step : -max_step;
      target_position += step;
      profile_vel = step / dt;
    }
    return;
  }

  if (fabsf(err) < 1e-5f) {
    target_position = angle_goal;
    profile_vel = 0.0f;
    return;
  }

  float dir = (err > 0.0f) ? 1.0f : -1.0f;
  float v = profile_vel;
  float stop_dist = (v * v) / (2.0f * fmaxf(a_dec, 1e-6f));
  bool wrong_way = (v * err) < 0.0f;
  bool must_brake = wrong_way || (fabsf(err) <= stop_dist);

  float v_cmd = v;
  if (must_brake) {
    float dv = a_dec * dt;
    if (fabsf(v) <= dv) {
      v_cmd = 0.0f;
    } else {
      v_cmd = v - ((v > 0.0f) ? dv : -dv);
    }
  } else {
    float v_tgt = dir * vmax;
    float dv = a_acc * dt;
    if (v < v_tgt) v_cmd = fminf(v + dv, v_tgt);
    else            v_cmd = fmaxf(v - dv, v_tgt);
  }

  float step = v_cmd * dt;
  if (fabsf(step) >= fabsf(err) && !wrong_way) {
    target_position = angle_goal;
    profile_vel = 0.0f;
  } else {
    target_position += step;
    profile_vel = v_cmd;
  }
}

/** Current foldback near soft ends + jam detect → motor disable. */
static void protectSoftEnds() {
  if (home_move || endstop_fault || !motor.enabled) return;

  float act = actualDeg();
  float des = desiredDeg();
  float lo = softMinDeg();
  float hi = softMaxDeg();
  float dist_lo = act - lo;
  float dist_hi = hi - act;
  bool near_end = (dist_lo < END_ZONE_DEG) || (dist_hi < END_ZONE_DEG);

  float cur_lim = MAX_CURRENT_A;
  if (near_end) cur_lim = FOLDBACK_CURRENT_A;
  if (motor.current_limit != cur_lim) {
    motor.current_limit = cur_lim;
    motor.PID_velocity.limit = cur_lim;
  }

  // Do NOT disable for "past soft" — after a hard-stop home the shaft often sits
  // slightly outside the soft window; that used to max-torque then trip.
  // Only trip on sustained stall while tracking a command well inside the window.
  float err = fabsf(des - act);
  float vel = fabsf(motor.shaft_velocity);
  float uq = fabsf(motor.voltage.q);
  bool cmd_inside = (des > lo + 2.0f) && (des < hi - 2.0f);
  bool jam_like = cmd_inside && (err > JAM_ERR_DEG) && (vel < JAM_VEL_RAD_S) && (uq > JAM_UQ_ABS);

  uint32_t now = millis();
  if (now < jam_grace_until) {
    jam_sense_t0 = 0;
    return;
  }
  if (jam_like) {
    if (jam_sense_t0 == 0) jam_sense_t0 = now;
    else if (now - jam_sense_t0 >= JAM_HOLD_MS) {
      tripEndstopFault("jam / endstop");
    }
  } else {
    jam_sense_t0 = 0;
  }
}

// SZ / SET_ZERO — like HALL_2 setZeroHere: shift zero and travel limits together.
static void setManualZero() {
  sensor.update();
  float sum = 0.0f;
  for (uint8_t i = 0; i < 20; i++) {
    sensor.update();
    sum += motor.shaft_angle;
    delay(1);
  }
  float before = wrapPI(motor.shaft_angle - mech_zero_offset) * RAD2DEG;
  mech_zero_offset = sum / 20.0f;
  travel_min_deg -= before;
  travel_max_deg -= before;
  hard_min_deg -= before;
  hard_max_deg -= before;
  cfg.soft_min_deg -= before;
  cfg.soft_max_deg -= before;
  applySoftWindow();
  applyAngleCommand(0.0f);
}

static void setZeroOffsetDeg(float deg) {
  mech_zero_offset = deg * DEG2RAD;
  applyAngleCommand(cmd_angle_deg);
}

// ---------------------------------------------------------------------------
// EEPROM — electrical calib is one-time; survive version bumps when magic matches
// ---------------------------------------------------------------------------
static bool configMagicOk() {
  return cfg.magic == ORBIT_CFG_MAGIC;
}

static bool electricalCalibUsable() {
  if (!cfg.calib_valid) return false;
  if (cfg.sensor_dir != 1 && cfg.sensor_dir != -1) return false;
  if (isnan(cfg.zero_electric_angle) || isinf(cfg.zero_electric_angle)) return false;
  return true;
}

static bool loadConfig() {
  EEPROM.get(0, cfg);
  return configMagicOk() && cfg.version == ORBIT_CFG_VERSION;
}

/** Load any matching-magic blob (any version). Returns true if magic OK. */
static bool loadConfigAnyVersion() {
  EEPROM.get(0, cfg);
  return configMagicOk();
}

static void saveConfig(bool calib_valid = true) {
  cfg.magic = ORBIT_CFG_MAGIC;
  cfg.version = ORBIT_CFG_VERSION;
  cfg.calib_valid = calib_valid ? 1 : 0;
  cfg.sensor_dir = (motor.sensor_direction == Direction::CCW) ? -1 : 1;
  cfg.zero_electric_angle = motor.zero_electric_angle;
  cfg.mech_zero_offset = mech_zero_offset;
  cfg.can_node_id = can_node;
  EEPROM.put(0, cfg);
}

static void setDefaultTuning() {
  cfg.vel_p    = ORBIT_DEF_VEL_P;
  cfg.vel_i    = ORBIT_DEF_VEL_I;
  cfg.vel_d    = ORBIT_DEF_VEL_D;
  cfg.vel_ramp = ORBIT_DEF_VEL_RAMP;
  cfg.angle_p  = ORBIT_DEF_ANGLE_P;
  cfg.lpf_tf   = ORBIT_DEF_LPF_TF;
  cfg.ekf_enabled = ORBIT_DEF_EKF_EN;
  cfg.ekf_q_angle = ORBIT_DEF_EKF_QA;
  cfg.ekf_q_vel   = ORBIT_DEF_EKF_QV;
  cfg.ekf_r_meas  = ORBIT_DEF_EKF_R;
  cfg.trq_p   = ORBIT_DEF_TRQ_P;
  cfg.trq_i   = ORBIT_DEF_TRQ_I;
  cfg.trq_d   = ORBIT_DEF_TRQ_D;
  cfg.trq_lpf = ORBIT_DEF_TRQ_LPF;
  cfg.slew_rate = ORBIT_DEF_SLEW;
  cfg.accel_rate = ORBIT_DEF_ACCEL;
  cfg.decel_rate = ORBIT_DEF_DECEL;
  cfg.soft_min_deg = ORBIT_DEF_SOFT_MIN;
  cfg.soft_max_deg = ORBIT_DEF_SOFT_MAX;
}

// Push tuning into motor + hall EKF.
static void applyTuning() {
  motor.PID_velocity.P = cfg.vel_p;
  motor.PID_velocity.I = cfg.vel_i;
  motor.PID_velocity.D = cfg.vel_d;
  motor.PID_velocity.output_ramp = cfg.vel_ramp;
  motor.PID_velocity.limit = motor.current_limit;  // estimated_current: A
  motor.P_angle.P = cfg.angle_p;
  motor.LPF_velocity.Tf = cfg.lpf_tf;

  sensor.ekf_enabled = (cfg.ekf_enabled != 0);
  sensor.q_angle = cfg.ekf_q_angle;
  sensor.q_vel = cfg.ekf_q_vel;
  sensor.r_meas = cfg.ekf_r_meas;

  motor.PID_current_q.P = cfg.trq_p;
  motor.PID_current_q.I = cfg.trq_i;
  motor.PID_current_q.D = cfg.trq_d;
  motor.PID_current_d.P = cfg.trq_p;
  motor.PID_current_d.I = cfg.trq_i;
  motor.PID_current_d.D = cfg.trq_d;
  motor.LPF_current_q.Tf = cfg.trq_lpf;
  motor.LPF_current_d.Tf = cfg.trq_lpf;

  // FOC velocity cap slightly above slew so the angle loop can track the ramp.
  if (cfg.slew_rate > 0.05f) {
    float vlim = cfg.slew_rate * DEG2RAD * 1.2f;
    if (vlim < 0.5f) vlim = 0.5f;
    motor.velocity_limit = vlim;
  } else {
    motor.velocity_limit = 30.0f;
  }
}

static float getParam(uint8_t idx) {
  switch (idx) {
    case ORBIT_PARAM_VEL_P:    return cfg.vel_p;
    case ORBIT_PARAM_VEL_I:    return cfg.vel_i;
    case ORBIT_PARAM_VEL_D:    return cfg.vel_d;
    case ORBIT_PARAM_VEL_RAMP: return cfg.vel_ramp;
    case ORBIT_PARAM_ANGLE_P:  return cfg.angle_p;
    case ORBIT_PARAM_LPF_TF:   return cfg.lpf_tf;
    case ORBIT_PARAM_EKF_EN:   return (float)cfg.ekf_enabled;
    case ORBIT_PARAM_EKF_QA:   return cfg.ekf_q_angle;
    case ORBIT_PARAM_EKF_QV:   return cfg.ekf_q_vel;
    case ORBIT_PARAM_EKF_R:    return cfg.ekf_r_meas;
    case ORBIT_PARAM_TRQ_P:    return cfg.trq_p;
    case ORBIT_PARAM_TRQ_I:    return cfg.trq_i;
    case ORBIT_PARAM_TRQ_D:    return cfg.trq_d;
    case ORBIT_PARAM_TRQ_LPF:  return cfg.trq_lpf;
    case ORBIT_PARAM_SLEW:     return cfg.slew_rate;
    case ORBIT_PARAM_ACCEL:    return cfg.accel_rate;
    case ORBIT_PARAM_DECEL:    return cfg.decel_rate;
    case ORBIT_PARAM_SOFT_MIN: return cfg.soft_min_deg;
    case ORBIT_PARAM_SOFT_MAX: return cfg.soft_max_deg;
    default: return 0.0f;
  }
}

static void setParam(uint8_t idx, float v) {
  if (isnan(v) || isinf(v)) return;
  switch (idx) {
    case ORBIT_PARAM_VEL_P:    cfg.vel_p    = constrain(v, 0.0f, 5.0f);       break;
    case ORBIT_PARAM_VEL_I:    cfg.vel_i    = constrain(v, 0.0f, 200.0f);     break;
    case ORBIT_PARAM_VEL_D:    cfg.vel_d    = constrain(v, 0.0f, 5.0f);       break;
    case ORBIT_PARAM_VEL_RAMP: cfg.vel_ramp = constrain(v, 1.0f, 1000000.0f); break;
    case ORBIT_PARAM_ANGLE_P:  cfg.angle_p  = constrain(v, 0.1f, 200.0f);     break;
    case ORBIT_PARAM_LPF_TF:   cfg.lpf_tf   = constrain(v, 0.0f, 2.0f);       break;
    case ORBIT_PARAM_EKF_EN:   cfg.ekf_enabled = (v != 0.0f) ? 1 : 0;         break;
    case ORBIT_PARAM_EKF_QA:   cfg.ekf_q_angle = constrain(v, 0.0f, 10.0f);   break;
    case ORBIT_PARAM_EKF_QV:   cfg.ekf_q_vel   = constrain(v, 0.0f, 1000.0f); break;
    case ORBIT_PARAM_EKF_R:    cfg.ekf_r_meas  = constrain(v, 1e-6f, 100.0f); break;
    case ORBIT_PARAM_TRQ_P:    cfg.trq_p    = constrain(v, 0.0f, 50.0f);      break;
    case ORBIT_PARAM_TRQ_I:    cfg.trq_i    = constrain(v, 0.0f, 1000.0f);    break;
    case ORBIT_PARAM_TRQ_D:    cfg.trq_d    = constrain(v, 0.0f, 10.0f);      break;
    case ORBIT_PARAM_TRQ_LPF:  cfg.trq_lpf  = constrain(v, 0.0f, 0.5f);       break;
    case ORBIT_PARAM_SLEW:     cfg.slew_rate = constrain(v, 0.0f, ORBIT_SLEW_MAX); break;
    case ORBIT_PARAM_ACCEL:    cfg.accel_rate = constrain(v, 0.0f, ORBIT_ACCEL_MAX); break;
    case ORBIT_PARAM_DECEL:    cfg.decel_rate = constrain(v, 0.0f, ORBIT_ACCEL_MAX); break;
    case ORBIT_PARAM_SOFT_MIN:
      cfg.soft_min_deg = v;
      applySoftWindow();
      applyAngleCommand(cmd_angle_deg);
      return;
    case ORBIT_PARAM_SOFT_MAX:
      cfg.soft_max_deg = v;
      applySoftWindow();
      applyAngleCommand(cmd_angle_deg);
      return;
    default: return;
  }
  applyTuning();
}

static void requestRecalibration() {
  // Keep tuning / node; clear electrical calib only, then reboot to re-align once.
  (void)loadConfigAnyVersion();
  cfg.magic = ORBIT_CFG_MAGIC;
  cfg.version = ORBIT_CFG_VERSION;
  cfg.calib_valid = 0;
  EEPROM.put(0, cfg);
  delay(50);
  NVIC_SystemReset();
}

static void setCanNode(uint8_t node) {
  if (node > ORBIT_CAN_NODE_MAX) return;
  Serial.print(F("SET_NODE "));
  Serial.print(can_node);
  Serial.print(F(" -> "));
  Serial.print(node);
  Serial.println(F(" (EEPROM + reboot)"));
  Serial.flush();
  cfg.can_node_id = node;
  saveConfig(cfg.calib_valid != 0);
  delay(80);
  NVIC_SystemReset();
}

// ---------------------------------------------------------------------------
// Boot home (velocity mode temporarily) — from HALL_2
// ---------------------------------------------------------------------------
static float home_vel_rad_s = 0.0f;
static float home_uq_cmd = 0.0f;
static float home_iq_cmd = 0.0f;
static float home_iq_target = 0.0f;
static float home_iq_now = 0.0f;
static uint32_t home_iq_us = 0;
static float home_hard_shaft = 0.0f;
static float home_hard_track = 0.0f;
static float home_cw_track = 0.0f;
static float home_ccw_track = 0.0f;
static float home_v_f = 0.0f;
static float home_i_term = 0.0f;
static float home_prev_rad = 0.0f;
static uint32_t home_spd_t = 0;
static bool home_boost_on = false;

/** Torque mode only. Speed is trimmed slowly in homeSpeedUpdate(). */
static void applyHomeDriveBoost() {
  home_boost_on = false;
  home_uq_cmd = 0.0f;
  home_vel_rad_s = 0.0f;
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 4.5f;
  motor.current_limit = MAX_CURRENT_A;
  motor.PID_velocity.reset();
  motor.P_angle.reset();
  Serial.println(F("HOME speed hold, brake on gravity"));
}

static void restoreNormalDriveLimits() {
  home_boost_on = false;
  home_uq_cmd = 0.0f;
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.controller = MotionControlType::angle;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.current_limit = MAX_CURRENT_A;
  motor.PID_velocity.limit = MAX_CURRENT_A;
}

static void enterVelocityHomeMode() {
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.PID_velocity.reset();
  motor.P_angle.reset();
}

static float homeUqForRpm(float rpm, float u_abs) {
  if (rpm > 0.0f) return fabsf(u_abs);
  if (rpm < 0.0f) return -fabsf(u_abs);
  return 0.0f;
}

static void setHomeCurrent(float amps) {
  if (amps > MAX_CURRENT_A) amps = MAX_CURRENT_A;
  if (amps < -MAX_CURRENT_A) amps = -MAX_CURRENT_A;
  home_iq_target = amps;
  home_uq_cmd = 0.0f;
  home_vel_rad_s = 0.0f;
}

/** Ramp the current actually applied. A step target still leaves the shaft smooth. */
static void homeMoveStep() {
  if (motor.controller != MotionControlType::torque) {
    motor.move(home_vel_rad_s);
    return;
  }
  uint32_t us = micros();
  float dt = (us - home_iq_us) * 1.0e-6f;
  home_iq_us = us;
  if (dt < 0.0f || dt > 0.05f) dt = 0.001f;
  float max_step = HOME_IQ_SLEW * dt;
  float err = home_iq_target - home_iq_now;
  if (err > max_step) err = max_step;
  if (err < -max_step) err = -max_step;
  home_iq_now += err;
  home_iq_cmd = home_iq_now;
  motor.move(home_iq_now);
}

static void homeSpeedInit(float preload_a) {
  home_v_f = 0.0f;
  home_i_term = preload_a;
  home_prev_rad = motor.shaft_angle;
  home_spd_t = millis();
  setHomeCurrent(preload_a);
}

/** 50 Hz trim: hold v_des (deg/s). Returns filtered shaft speed. */
static float homeSpeedUpdate(float v_des, float i_cap, float* travel) {
  uint32_t now = millis();
  float dt = (now - home_spd_t) * 0.001f;
  if (dt < 0.02f) return home_v_f;
  if (dt > 0.05f) dt = 0.02f;
  home_spd_t = now;
  float ang = motor.shaft_angle;
  float step = wrapPI(ang - home_prev_rad) * RAD2DEG;
  home_prev_rad = ang;
  if (travel) *travel += fabsf(step);
  float v = step / dt;
  home_v_f += 0.20f * (v - home_v_f);
  float verr = v_des - home_v_f;
  // Same gain both ways. Deadband ignores hall noise.
  float p = 0.0f;
  if (fabsf(verr) > 12.0f) {
    p = 0.003f * verr;
    if (p > 0.30f) p = 0.30f;
    if (p < -0.30f) p = -0.30f;
  }
  home_i_term += 0.06f * verr * dt;
  if (i_cap < 0.04f) i_cap = 0.04f;
  if (i_cap > MAX_CURRENT_A) i_cap = MAX_CURRENT_A;
  if (home_i_term > i_cap) home_i_term = i_cap;
  if (home_i_term < -i_cap) home_i_term = -i_cap;
  setHomeCurrent(home_i_term + p);
  return home_v_f;
}

/** Ease current to zero instead of cutting it. */
static void homeRelease() {
  setHomeCurrent(0.0f);
  uint32_t t0 = millis();
  while (millis() - t0 < 700UL) {
    motor.loopFOC();
    homeMoveStep();
    if (fabsf(home_iq_now) < 0.015f) break;
  }
  home_iq_now = 0.0f;
  home_iq_target = 0.0f;
  home_iq_cmd = 0.0f;
  home_i_term = 0.0f;
  motor.move(0.0f);
  motor.setPhaseVoltage(0.0f, 0.0f, motor.electrical_angle);
}

static void homeCoast() {
  homeRelease();
}

static void restoreAngleMode() {
  restoreNormalDriveLimits();
  motor.controller = MotionControlType::angle;
  motor.PID_velocity.reset();
  motor.P_angle.reset();
  applyTuning();
  sensor.update();
  target_position = motor.shaft_angle;
  angle_goal = motor.shaft_angle;
}

/** Hold exact shaft angle now — never yank toward soft mid / 0°. */
static void holdShaftHere() {
  home_move = false;
  vel_mode = false;
  clearPidIntegrals();
  restoreAngleMode();
  sensor.update();
  target_position = motor.shaft_angle;
  angle_goal = motor.shaft_angle;
  cmd_angle_deg = actualDeg();
  profile_vel = 0.0f;
  Serial.print(F("  HOLD shaft @ "));
  Serial.print(cmd_angle_deg, 1);
  Serial.println(F(" deg"));
}

static void applyVelCommand(float rpm) {
  if (endstop_fault || !motor.enabled) return;
  if (fabsf(rpm) < 0.05f) {
    vel_mode = false;
    sensor.update();
    cmd_angle_deg = actualDeg();
    restoreAngleMode();
    return;
  }
  vel_mode = true;
  enterVelocityHomeMode();
  applyTuning();
  rpm = constrain(rpm, -60.0f, 60.0f);
  target_vel_mech = rpm * (_2PI / 60.0f);
}

static float rpmToRadS(float rpm) {
  return rpm * (_2PI / 60.0f);
}

static void homeTrackReset() {
  motor.loopFOC();
  home_track_prev = motor.shaft_angle;
  home_track_deg = 0.0f;
}

static void homeTrackStep() {
  float a = motor.shaft_angle;
  home_track_deg += wrapPI(a - home_track_prev) * RAD2DEG;
  home_track_prev = a;
}

/**
 * Drive one direction and record the furthest shaft angle.
 * The hard stop is that extreme, not a speed guess. Current is cut
 * once the extreme stops advancing, so it does not keep slamming.
 */
static void homeUseNormalPid() {
  restoreNormalDriveLimits();
  motor.controller = MotionControlType::angle;
  applyTuning();
  sensor.update();
  target_position = motor.shaft_angle;
  angle_goal = target_position;
  profile_vel = 0.0f;
  motor.loopFOC();
  motor.move(target_position);
}

/** Move the normal angle target. Speed slews at accel when leaving a stop. */
static void homeRampSpeed(float v_target, float accel, float dt) {
  if (dt < 1e-4f) dt = 1e-4f;
  float dv = accel * dt;
  if (home_v_cmd < v_target) {
    home_v_cmd += dv;
    if (home_v_cmd > v_target) home_v_cmd = v_target;
  } else if (home_v_cmd > v_target) {
    home_v_cmd -= dv;
    if (home_v_cmd < v_target) home_v_cmd = v_target;
  }
  float err = home_cmd_track - home_track_deg;
  if (fabsf(err) < 15.0f) {
    float step = home_v_cmd * dt;
    home_cmd_track += step;
    target_position += step * DEG2RAD;
  }
  angle_goal = target_position;
  motor.move(target_position);
}

static bool driveUntilStall(float rpm, uint32_t timeout_ms, bool reverse_from_stop) {
  const float sign = (rpm >= 0.0f) ? 1.0f : -1.0f;
  const float v_target = sign * HOME_CRUISE_DPS;
  home_move = true;
  if (motor.controller != MotionControlType::angle) homeUseNormalPid();
  motor.loopFOC();
  homeTrackStep();
  home_cmd_track = home_track_deg;
  home_v_cmd = reverse_from_stop ? 0.0f : v_target;
  float start_track = home_track_deg;
  float extreme_track = start_track;
  uint32_t still_ms = millis();
  uint32_t t0 = still_ms;
  uint32_t last = t0;
  const float accel = reverse_from_stop ? HOME_REVERSE_ACCEL : 10000.0f;
  while (millis() - t0 < timeout_ms) {
    uint32_t now = millis();
    float dt = (now - last) * 0.001f;
    last = now;
    if (dt < 0.0f || dt > 0.05f) dt = 0.002f;
    motor.loopFOC();
    homeTrackStep();
    homeRampSpeed(v_target, accel, dt);
    float gained = (home_track_deg - extreme_track) * sign;
    if (gained > 0.4f) {
      extreme_track = home_track_deg;
      home_hard_shaft = motor.shaft_angle;
      home_hard_track = extreme_track;
      still_ms = now;
    }
    float travel_deg = fabsf(extreme_track - start_track);
    float back = (extreme_track - home_track_deg) * sign;
    if (travel_deg >= 20.0f && back < 1.5f && (now - still_ms) > HOME_ARREST_MS) {
      home_hard_track = extreme_track;
      home_v_cmd = 0.0f;
      target_position = motor.shaft_angle;
      angle_goal = target_position;
      motor.move(target_position);
      Serial.print(F("  END track="));
      Serial.print(extreme_track, 1);
      Serial.print(F("  travel="));
      Serial.print(travel_deg, 1);
      Serial.println(F(" deg"));
      return true;
    }
    if (travel_deg < 4.0f && (now - t0) > HOME_NO_MOVE_MS) {
      Serial.println(F("  no movement — not an end"));
      return false;
    }
  }
  if (fabsf(extreme_track - start_track) >= 20.0f) {
    home_hard_track = extreme_track;
    home_v_cmd = 0.0f;
    Serial.print(F("  END on timeout track="));
    Serial.println(extreme_track, 1);
    return true;
  }
  Serial.println(F("  END TIMEOUT"));
  return false;
}

/** Move a measured distance off a recorded hard stop. */
static void backOffFrom(float hard_rad, float sign_away) {
  (void)hard_rad;
  home_move = true;
  if (motor.controller != MotionControlType::angle) homeUseNormalPid();
  motor.loopFOC();
  homeTrackStep();
  home_cmd_track = home_track_deg;
  home_v_cmd = 0.0f;
  float start = home_track_deg;
  const float v_target = sign_away * HOME_CREEP_DPS;
  uint32_t t0 = millis();
  uint32_t last = t0;
  while (millis() - t0 < 3000UL) {
    uint32_t now = millis();
    float dt = (now - last) * 0.001f;
    last = now;
    if (dt < 0.0f || dt > 0.05f) dt = 0.002f;
    motor.loopFOC();
    homeTrackStep();
    homeRampSpeed(v_target, HOME_REVERSE_ACCEL, dt);
    if (fabsf(home_track_deg - start) >= 8.0f) break;
  }
}

/** Drive to the pitch zero at the same slow speed, easing off as it arrives. */
static bool driveToShaftRad(float target_shaft_rad, float unused_rpm, uint32_t timeout_ms) {
  (void)unused_rpm;
  (void)target_shaft_rad;
  home_move = true;
  if (motor.controller != MotionControlType::angle) homeUseNormalPid();
  motor.loopFOC();
  homeTrackStep();
  home_cmd_track = home_track_deg;
  home_v_cmd = 0.0f;
  float target_track = home_zero_track;
  uint32_t t0 = millis();
  uint32_t last = t0;
  bool ok = false;
  while (millis() - t0 < timeout_ms) {
    uint32_t now = millis();
    float dt = (now - last) * 0.001f;
    last = now;
    if (dt < 0.0f || dt > 0.05f) dt = 0.002f;
    motor.loopFOC();
    homeTrackStep();
    float left = target_track - home_track_deg;
    if (fabsf(left) < 2.0f) {
      ok = true;
      break;
    }
    float v_target = (left > 0.0f) ? HOME_CRUISE_DPS : -HOME_CRUISE_DPS;
    if (fabsf(left) < 15.0f) v_target = left * 4.0f;
    homeRampSpeed(v_target, HOME_REVERSE_ACCEL, dt);
  }
  target_position = motor.shaft_angle;
  angle_goal = target_position;
  motor.loopFOC();
  motor.move(target_position);
  return ok || fabsf(target_track - home_track_deg) < HOME_MID_OK_DEG;
}

static void bootHomeCwCcwMid() {
  Serial.println(F("BOOT HOME — hard span around 270, zero at mid"));
  home_move = true;
  jam_grace_until = millis() + 60000UL;
  homeUseNormalPid();

  homeTrackReset();
  float rpm_pos = HOME_SEEK_RPM;
  float rpm_neg = -HOME_SEEK_RPM;

  // Positive shaft direction is CW on this pitch axis.
  bool hit_cw = driveUntilStall(rpm_pos, HOME_TIMEOUT_MS, false);
  float cw_shaft = home_hard_shaft;
  float cw_track = home_hard_track;
  home_cw_track = cw_track;
  Serial.print(F("  hard CW="));
  Serial.println(hit_cw ? F("OK") : F("FAIL"));

  bool hit_ccw = driveUntilStall(rpm_neg, HOME_TIMEOUT_MS, true);
  float ccw_track = home_hard_track;
  home_ccw_track = ccw_track;
  Serial.print(F("  hard CCW="));
  Serial.println(hit_ccw ? F("OK") : F("FAIL"));

  float span_deg = fabsf(home_cw_track - home_ccw_track);
  Serial.print(F("  HARD span="));
  Serial.print(span_deg, 1);
  Serial.print(F("  CW="));
  Serial.print(home_cw_track, 1);
  Serial.print(F("  CCW="));
  Serial.print(home_ccw_track, 1);
  Serial.println(F(" deg (accept 200-340)"));

  float err_deg = 999.0f;
  home_move = true;

  if (!hit_cw || !hit_ccw || !adoptPitchFromTracks(home_ccw_track, home_cw_track)) {
    Serial.println(F("HOME rejected — zero unchanged"));
    if (hit_cw && !hit_ccw) backOffFrom(cw_shaft, -1.0f);
    hard_limits_valid = false;
    homed = false;
    holdShaftHere();
  } else {
    float zero_shaft = mech_zero_offset;

    Serial.println(F("  slow drive to pitch 0"));
    (void)driveToShaftRad(zero_shaft, 0.0f, HOME_MID_MS);
    sensor.update();
    err_deg = fabsf(motor.shaft_angle - zero_shaft) * RAD2DEG;
    if (err_deg > HOME_MID_OK_DEG) {
      (void)driveToShaftRad(zero_shaft, 0.0f, HOME_MID_MS);
      sensor.update();
      err_deg = fabsf(motor.shaft_angle - zero_shaft) * RAD2DEG;
    }

    // Re-apply soft (zero unchanged) after approach
    (void)adoptPitchFromTracks(home_ccw_track, home_cw_track);
    homed = (err_deg < HOME_MID_OK_DEG);

    Serial.print(F("  mid_err="));
    Serial.print(err_deg, 1);
    Serial.print(F("  homed="));
    Serial.println(homed ? F("yes") : F("no"));

    holdShaftHere();
    if (homed) {
      applyAngleCommand(0.0f);
      Serial.println(F("  at 0 — hold"));
    } else {
      Serial.println(F("  0 not reached — staying on hold (no yank)"));
    }
  }

  home_move = false;
  jam_grace_until = millis() + JAM_GRACE_MS;
  Serial.print(F("HOME done  soft["));
  Serial.print(travel_min_deg, 1);
  Serial.print(F(","));
  Serial.print(travel_max_deg, 1);
  Serial.print(F("]  hard_span="));
  Serial.print(span_deg, 1);
  Serial.print(F("  node="));
  Serial.print(can_node);
  Serial.print(F("  homed="));
  Serial.println(homed ? F("yes") : F("no"));
}

// ---------------------------------------------------------------------------
// Serial protocol (same pattern as OrbitDrive3.0)
// ---------------------------------------------------------------------------
static char rx_line[64];
static uint8_t rx_len = 0;
static uint8_t gw_pan_node  = ORBIT_GW_PAN_NODE;
static uint8_t gw_tilt_node = ORBIT_GW_TILT_NODE;

static void sendAngleToNode(uint8_t node, float deg) {
  deg = constrain(deg, softMinDeg(), softMaxDeg());
  if (node == can_node) {
    applyAngleCommand(deg);
  }
  if (!can_ready) return;
  CAN_message_t tx;
  memset(&tx, 0, sizeof(tx));
  tx.id = ORBIT_CAN_CMD_ID(node, ORBIT_CMD_SET_ANGLE);
  tx.flags.extended = 0;
  tx.flags.remote = 0;
  tx.len = 4;
  memcpy(tx.buf, &deg, 4);
  Can1.write(tx);
}

static void gatewaySetPanTilt(float pan_deg, float tilt_deg) {
  sendAngleToNode(gw_pan_node, pan_deg);
  sendAngleToNode(gw_tilt_node, tilt_deg);
  Serial.print(F("ACK PT "));
  Serial.print(constrain(pan_deg, softMinDeg(), softMaxDeg()), 2);
  Serial.print(' ');
  Serial.println(constrain(tilt_deg, softMinDeg(), softMaxDeg()), 2);
}

static void reportConfig() {
  Serial.print(F("CFG,"));
  Serial.print(mech_zero_offset, 5); Serial.print(',');
  Serial.print(motor.zero_electric_angle, 5); Serial.print(',');
  Serial.print(motor.sensor_direction == Direction::CCW ? -1 : 1); Serial.print(',');
  Serial.print(softMinDeg(), 1); Serial.print(',');
  Serial.print(softMaxDeg(), 1); Serial.print(',');
  Serial.println(mech_zero_offset * RAD2DEG, 3);
  Serial.print(F("NODE,")); Serial.println(can_node);
  Serial.print(F("HOME,"));
  Serial.print(homed ? 1 : 0); Serial.print(',');
  Serial.print(travel_min_deg, 1); Serial.print(',');
  Serial.println(travel_max_deg, 1);
}

static void reportTuning() {
  Serial.print(F("PID,"));
  Serial.print(cfg.vel_p, 4);    Serial.print(',');
  Serial.print(cfg.vel_i, 4);    Serial.print(',');
  Serial.print(cfg.vel_d, 4);    Serial.print(',');
  Serial.print(cfg.vel_ramp, 1); Serial.print(',');
  Serial.print(cfg.angle_p, 4);  Serial.print(',');
  Serial.println(cfg.lpf_tf, 5);
  Serial.print(F("EKF,"));
  Serial.print(cfg.ekf_enabled);   Serial.print(',');
  Serial.print(cfg.ekf_q_angle, 6); Serial.print(',');
  Serial.print(cfg.ekf_q_vel, 4);   Serial.print(',');
  Serial.println(cfg.ekf_r_meas, 6);
  Serial.print(F("TRQ,"));
  Serial.print(cfg.trq_p, 4);   Serial.print(',');
  Serial.print(cfg.trq_i, 4);   Serial.print(',');
  Serial.print(cfg.trq_d, 4);   Serial.print(',');
  Serial.println(cfg.trq_lpf, 5);
  Serial.print(F("SLEW,"));
  Serial.println(cfg.slew_rate, 1);
  Serial.print(F("ACCEL,"));
  Serial.println(cfg.accel_rate, 1);
  Serial.print(F("DECEL,"));
  Serial.println(cfg.decel_rate, 1);
}

static void handleLine(char *line) {
  if (line[0] == 'P' && line[1] == 'T') {
    char *p = line + 2;
    float pan = strtof(p, &p);
    float tilt = strtof(p, NULL);
    gatewaySetPanTilt(pan, tilt);
    return;
  }
  if (line[0] == 'P' && line[1] == 'A' && line[2] == 'N') {
    float pan = strtof(line + 3, NULL);
    sendAngleToNode(gw_pan_node, pan);
    Serial.print(F("ACK PAN "));
    Serial.println(constrain(pan, softMinDeg(), softMaxDeg()), 2);
    return;
  }
  if (line[0] == 'T' && line[1] == 'I' && line[2] == 'L' && line[3] == 'T') {
    float tilt = strtof(line + 4, NULL);
    sendAngleToNode(gw_tilt_node, tilt);
    Serial.print(F("ACK TILT "));
    Serial.println(constrain(tilt, softMinDeg(), softMaxDeg()), 2);
    return;
  }
  if (line[0] == 'S' && line[1] == 'A') {
    applyAngleCommand(atof(line + 2));
    Serial.print(F("ACK SA ")); Serial.println(cmd_angle_deg, 2);
  } else if (line[0] == 'S' && line[1] == 'Z') {
    setManualZero();
    Serial.println(F("ACK SZ"));
  } else if (line[0] == 'S' && line[1] == 'V') {
    saveConfig();
    Serial.println(F("ACK SV"));
  } else if (line[0] == 'S' && line[1] == 'O') {
    setZeroOffsetDeg(atof(line + 2));
    Serial.print(F("ACK SO ")); Serial.println(mech_zero_offset * RAD2DEG, 3);
  } else if (line[0] == 'S' && line[1] == 'C') {
    Serial.println(F("ACK SC — recalibrating, rebooting"));
    Serial.flush();
    requestRecalibration();
  } else if (line[0] == 'S' && line[1] == 'N') {
    int id = (int)strtol(line + 2, NULL, 10);
    Serial.print(F("ACK SN ")); Serial.print(id);
    Serial.println(F(" — saving + rebooting")); Serial.flush();
    setCanNode((uint8_t)id);
  } else if (line[0] == 'S' && line[1] == 'P') {
    char *p = line + 2;
    int idx = (int)strtol(p, &p, 10);
    float val = strtof(p, NULL);
    setParam((uint8_t)idx, val);
    Serial.print(F("ACK SP ")); Serial.print(idx);
    Serial.print(' '); Serial.println(val, 5);
  } else if (line[0] == 'S' && line[1] == 'E') {
    int en = (int)strtol(line + 2, NULL, 10);
    setMotorEnable(en != 0);
    Serial.print(F("ACK SE ")); Serial.println(en != 0 ? 1 : 0);
  } else if (line[0] == 'S' && line[1] == 'G') {
    reportConfig();
    reportTuning();
    Serial.print(F("GW,")); Serial.print(gw_pan_node);
    Serial.print(','); Serial.println(gw_tilt_node);
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_len > 0) { rx_line[rx_len] = '\0'; handleLine(rx_line); rx_len = 0; }
    } else if (rx_len < sizeof(rx_line) - 1) {
      rx_line[rx_len++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------
static void sendParamsCan() {
  if (!can_ready) return;
  for (uint8_t i = 0; i < ORBIT_PARAM_COUNT; i++) {
    CAN_message_t tx;
    tx.id = can_id_param;
    tx.len = 5;
    tx.buf[0] = i;
    float v = getParam(i);
    memcpy(&tx.buf[1], &v, 4);
    Can1.write(tx);
    delay(1);
  }
}

static void pollCan() {
  if (!can_ready) return;
  CAN_message_t rx;
  while (Can1.read(rx)) {
    uint16_t off = rx.id - can_cmd_base;
    switch (off) {
      case ORBIT_CMD_SET_ANGLE:
        if (rx.len >= 4) { float d; memcpy(&d, rx.buf, 4); applyAngleCommand(d); }
        break;
      case ORBIT_CMD_SET_ZERO:
        setManualZero();
        break;
      case ORBIT_CMD_SAVE_CFG:
        saveConfig();
        break;
      case ORBIT_CMD_SET_OFFSET:
        if (rx.len >= 4) { float d; memcpy(&d, rx.buf, 4); setZeroOffsetDeg(d); }
        break;
      case ORBIT_CMD_RECALIBRATE:
        requestRecalibration();
        break;
      case ORBIT_CMD_SET_PARAM:
        if (rx.len >= 5) { float v; memcpy(&v, &rx.buf[1], 4); setParam(rx.buf[0], v); }
        break;
      case ORBIT_CMD_GET_PARAMS:
        sendParamsCan();
        break;
      case ORBIT_CMD_SET_NODE:
        if (rx.len >= 1) setCanNode(rx.buf[0]);
        break;
      case ORBIT_CMD_SET_VEL:
        if (rx.len >= 4) { float rpm; memcpy(&rpm, rx.buf, 4); applyVelCommand(rpm); }
        break;
      case ORBIT_CMD_SET_ENABLE:
        if (rx.len >= 1) setMotorEnable(rx.buf[0] != 0);
        break;
      default:
        break;
    }
  }
}

static void sendTelemetry() {
  float d = desiredDeg();
  float a = actualDeg();
  Serial.print(F("TEL,"));
  Serial.print(d, 2); Serial.print(',');
  Serial.println(a, 2);
  if (!can_ready) return;

  CAN_message_t tx;

  tx.id = can_id_telem;
  tx.len = 8;
  memcpy(&tx.buf[0], &d, 4);
  memcpy(&tx.buf[4], &a, 4);
  Can1.write(tx);

  float adc[3];
  hallAdcFiltered(adc);
  // Convert ADC counts → millivolts (3.3 V / 4095)
  uint16_t hu = (uint16_t)constrain(adc[0] * (3300.0f / 4095.0f) + 0.5f, 0.0f, 3300.0f);
  uint16_t hv = (uint16_t)constrain(adc[1] * (3300.0f / 4095.0f) + 0.5f, 0.0f, 3300.0f);
  uint16_t hw = (uint16_t)constrain(adc[2] * (3300.0f / 4095.0f) + 0.5f, 0.0f, 3300.0f);
  uint16_t pad = 0;
  tx.id = can_id_halls;
  tx.len = 8;
  memcpy(&tx.buf[0], &hu, 2);
  memcpy(&tx.buf[2], &hv, 2);
  memcpy(&tx.buf[4], &hw, 2);
  memcpy(&tx.buf[6], &pad, 2);
  Can1.write(tx);

  float uq = motor.voltage.q;
  float vel = motor.shaft_velocity;
  tx.id = can_id_status;
  tx.len = 8;
  memcpy(&tx.buf[0], &uq, 4);
  memcpy(&tx.buf[4], &vel, 4);
  Can1.write(tx);

  float lo = softMinDeg();
  float hi = softMaxDeg();
  tx.id = can_id_limits;
  tx.len = 8;
  memcpy(&tx.buf[0], &lo, 4);
  memcpy(&tx.buf[4], &hi, 4);
  Can1.write(tx);

  // Announce this drive's CAN node id (for UI scan / multi-drive setup)
  tx.id = can_id_node;
  tx.len = 8;
  memset(tx.buf, 0, 8);
  tx.buf[0] = can_node;
  Can1.write(tx);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  delay(500);
  Serial.println(F("=== OrbitDrive_HALL_5 boot ==="));

  // 1. Hall + GenericSensor + driver
  hallInit();
  sensor.init();
  for (int i = 0; i < 16; i++) { sensor.update(); delay(2); }
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  if (!driver.init()) {
    Serial.println(F("ERROR: driver.init"));
    while (true) blinkStatus(1, 80, 80);
  }
  motor.linkDriver(&driver);

  // FOC config — identical control loop to OrbitDrive3.0
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  motor.KV_rating = MOTOR_KV;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.current_limit = MAX_CURRENT_A;
  motor.velocity_limit = 30.0f;

  // 2. EEPROM: reuse one-time electrical calib; migrate across CFG version bumps
  bool have_store = loadConfigAnyVersion();
  bool version_ok = have_store && (cfg.version == ORBIT_CFG_VERSION);
  bool use_calib = have_store && electricalCalibUsable();
  mech_zero_offset = 0.0f;  // boot home always redefines mechanical zero

  if (!have_store) {
    memset(&cfg, 0, sizeof(cfg));
    setDefaultTuning();
    cfg.can_node_id = ORBIT_DEF_CAN_NODE;
    cfg.calib_valid = 0;
    Serial.println(F("Config: empty EEPROM — defaults + will FOC-align once"));
  } else if (!version_ok) {
    uint16_t from_ver = cfg.version;
    Serial.print(F("Config: migrating EEPROM v"));
    Serial.print(from_ver);
    Serial.print(F(" -> v"));
    Serial.println(ORBIT_CFG_VERSION);
    if (from_ver < 15) cfg.slew_rate = ORBIT_DEF_SLEW;
    if (from_ver < 17) {
      cfg.accel_rate = ORBIT_DEF_ACCEL;
      cfg.decel_rate = ORBIT_DEF_DECEL;
    }
    if (from_ver < 18) {
      // New default tuning pack — PID + profile (does not touch electrical calib)
      cfg.vel_p = ORBIT_DEF_VEL_P;
      cfg.vel_i = ORBIT_DEF_VEL_I;
      cfg.vel_d = ORBIT_DEF_VEL_D;
      cfg.vel_ramp = ORBIT_DEF_VEL_RAMP;
      cfg.angle_p = ORBIT_DEF_ANGLE_P;
      cfg.lpf_tf = ORBIT_DEF_LPF_TF;
      cfg.trq_p = ORBIT_DEF_TRQ_P;
      cfg.trq_i = ORBIT_DEF_TRQ_I;
      cfg.trq_d = ORBIT_DEF_TRQ_D;
      cfg.trq_lpf = ORBIT_DEF_TRQ_LPF;
      cfg.slew_rate = ORBIT_DEF_SLEW;
      cfg.accel_rate = ORBIT_DEF_ACCEL;
      cfg.decel_rate = ORBIT_DEF_DECEL;
    }
    if (from_ver < 19) {
      cfg.can_node_id = ORBIT_DEF_CAN_NODE;
    }
    if (from_ver < 20) {
      cfg.soft_min_deg = ORBIT_DEF_SOFT_MIN;
      cfg.soft_max_deg = ORBIT_DEF_SOFT_MAX;
    }
    if (from_ver < 21) {
      cfg.can_node_id = ORBIT_DEF_CAN_NODE;  // pitch/tilt -> node 1
      cfg.soft_min_deg = ORBIT_DEF_SOFT_MIN; // -45
      cfg.soft_max_deg = ORBIT_DEF_SOFT_MAX; // +90
      Serial.println(F("Pitch/tilt: CAN node 1, soft -45/+90"));
    }
    if (from_ver < 23) {
      cfg.soft_min_deg = ORBIT_DEF_SOFT_MIN;
      cfg.soft_max_deg = ORBIT_DEF_SOFT_MAX;
      Serial.println(F("Soft window opened to -135/+135"));
    }
    cfg.version = ORBIT_CFG_VERSION;
  } else {
    Serial.println(F("Config: EEPROM OK"));
  }

  applyCanNode(cfg.can_node_id);
  cfg.soft_min_deg = ORBIT_DEF_SOFT_MIN;
  cfg.soft_max_deg = ORBIT_DEF_SOFT_MAX;
  applyTuning();
  applySoftWindow();
  Serial.print(F("CAN node id: ")); Serial.println(can_node);
  Serial.print(F("Tuning: "));
  Serial.println(have_store ? F("from EEPROM") : F("defaults"));

  if (use_calib) {
    motor.sensor_direction = (cfg.sensor_dir < 0) ? Direction::CCW : Direction::CW;
    motor.zero_electric_angle = cfg.zero_electric_angle;
    Serial.print(F("Electrical calib: STORED  dir="));
    Serial.print(cfg.sensor_dir);
    Serial.print(F("  zero_el="));
    Serial.println(cfg.zero_electric_angle, 4);
  } else {
    motor.sensor_direction = Direction::UNKNOWN;
    motor.zero_electric_angle = NOT_SET;
    Serial.println(F("Electrical calib: NONE — FOC align this boot, then save"));
  }

  if (!use_calib) {
    // initFOC applies this as volts, not amps. 0.5 A needs I*R.
    float align_v = MAX_CURRENT_A * MOTOR_PHASE_RESISTANCE;
    if (align_v > DRIVER_VOLTAGE_LIMIT) align_v = DRIVER_VOLTAGE_LIMIT;
    motor.voltage_limit = align_v;
    motor.voltage_sensor_align = align_v;
    Serial.print(F("Align voltage "));
    Serial.println(align_v, 2);
  }

  if (!motor.init()) {
    Serial.println(F("ERROR: motor.init"));
    while (true) blinkStatus(2, 100, 200);
  }

  // 3. initFOC (skips long align when sensor_direction + zero_electric are set)
  setStatusLed(true);
  Serial.println(use_calib ? F("initFOC (using stored electrical calib)...")
                           : F("initFOC (aligning — one-time)..."));
  if (!motor.initFOC()) {
    Serial.println(F("ERROR: initFOC"));
    setStatusLed(false);
    motor.disable();
    while (true) { blinkStatus(5, 60, 60); delay(400); }
  }
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;

  if (!use_calib) {
    saveConfig(true);
    Serial.println(F("Electrical calib: SAVED to EEPROM (one-time)"));
  } else if (!version_ok) {
    // Persist migrated version without re-aligning
    saveConfig(true);
    Serial.println(F("Config: migrated EEPROM written"));
  }

  startupChime();

  // 4. Every boot: find hard limits → mid = mechanical zero
#if BOOT_AUTO_HOME
  bootHomeCwCcwMid();
#else
  sensor.update();
  mech_zero_offset = motor.shaft_angle;
  applyAngleCommand(0.0f);
#endif

  // 5. Angle hold at 0 (already applied in home / above)

  // 6. CAN + serial ready
  {
    RCC_PeriphCLKInitTypeDef fdcan_clk = {};
    fdcan_clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    fdcan_clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&fdcan_clk);
  }

  Can1.setFrameFormat(STM32_CAN::CLASSIC);
  can_ready = Can1.begin(ORBIT_CAN_BAUDRATE);
  if (can_ready) {
    Can1.setFilterSingleMask(0, can_cmd_base, 0x7F0, STD);
    Serial.print(F("CAN: up @ 500k, node ")); Serial.print(can_node);
    Serial.print(F(", cmd base 0x")); Serial.println(can_cmd_base, HEX);
    blinkStatus(2, 60, 140);
  } else {
    Serial.println(F("CAN: init failed"));
    blinkStatus(6, 60, 60);
  }

  setStatusLed(true);
  Serial.println(F("READY"));
  reportConfig();
  reportTuning();
}

void loop() {
  motor.loopFOC();
  if (home_move) {
    homeMoveStep();
  } else if (endstop_fault || !motor.enabled) {
    // Drive disabled — do not command torque
  } else if (vel_mode) {
    motor.move(target_vel_mech);
  } else {
    static uint32_t last_us = micros();
    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;
    stepAngleSlew(dt);
    protectSoftEnds();
    if (!endstop_fault && motor.enabled) {
      motor.move(target_position);
    }
  }

  pollSerial();
  pollCan();

  static uint32_t last_tel = 0;
  uint32_t now = millis();
  if (now - last_tel >= 20) {
    last_tel = now;
    sendTelemetry();
  }

  setStatusLed(motor.enabled && !endstop_fault);
}
