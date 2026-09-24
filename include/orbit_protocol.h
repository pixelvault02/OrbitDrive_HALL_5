#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Orbit Drive shared protocol / config definitions (HALL_5)
// Same serial/CAN command pattern as OrbitDrive3.0.
// Soft angle limits: user-editable (EEPROM) + boot home hard-span clamp.
// Pitch/tilt soft window: CCW -45° / CW +90°.
// Hard stops are around 270° end to end (about 135° each side of center).
// FOC calib once in EEPROM.
// ---------------------------------------------------------------------------

// Default / fallback angle command limits (degrees) — pitch/tilt asymmetric.
#define ORBIT_ANGLE_MIN_DEG (-135.0f)
#define ORBIT_ANGLE_MAX_DEG 135.0f

// CAN addressing (standard 11-bit identifiers) --------------------------------
// Each Orbit Drive on the bus must use a unique node id (0..15).
//   CMD IDs : 0x140 + node*0x10 + cmd_off   (host -> drive)
//   RPT IDs : 0x240 + node*0x10 + rpt_off   (drive -> host)
// Example node 1: SET_ANGLE=0x151, TELEMETRY=0x241
// Example node 2: SET_ANGLE=0x161, TELEMETRY=0x251
#define ORBIT_CAN_BAUDRATE     500000UL
#define ORBIT_CAN_NODE_MAX     15
#define ORBIT_CAN_NODE_STRIDE  0x10
#define ORBIT_CAN_CMD_BASE     0x140
#define ORBIT_CAN_RPT_BASE     0x240
#define ORBIT_CAN_CMD_ID(node, off) (ORBIT_CAN_CMD_BASE + (uint16_t)(node) * ORBIT_CAN_NODE_STRIDE + (off))
#define ORBIT_CAN_RPT_ID(node, off) (ORBIT_CAN_RPT_BASE + (uint16_t)(node) * ORBIT_CAN_NODE_STRIDE + (off))
#define ORBIT_CAN_CMD_FILTER(node)  (ORBIT_CAN_CMD_BASE + (uint16_t)(node) * ORBIT_CAN_NODE_STRIDE)

// Command offsets (added to a node's command base) — RX <- host
#define ORBIT_CMD_SET_ANGLE    0x01   // float32 degrees
#define ORBIT_CMD_SET_ZERO     0x02   // no payload (zero = current)
#define ORBIT_CMD_SAVE_CFG     0x03   // no payload (persist config)
#define ORBIT_CMD_SET_OFFSET   0x04   // float32 deg (zero offset from sensor 0)
#define ORBIT_CMD_RECALIBRATE  0x05   // no payload (re-align + save, reboots)
#define ORBIT_CMD_SET_PARAM    0x06   // uint8 param index + float32 value
#define ORBIT_CMD_GET_PARAMS   0x07   // no payload (request all params)
#define ORBIT_CMD_SET_NODE     0x08   // uint8 new node id (0..15); saves + reboots
#define ORBIT_CMD_SET_VEL      0x09   // float32 mechanical rpm (0 = return to angle hold)
#define ORBIT_CMD_SET_ENABLE   0x0A   // uint8: 0=disable, 1=enable (clears endstop fault)

// Readout offsets (added to a node's readout base) — TX -> host
#define ORBIT_RPT_TELEMETRY    0x01   // float32 desired + float32 actual (deg)
#define ORBIT_RPT_PARAM        0x02   // uint8 param index + float32 value
#define ORBIT_RPT_HALLS        0x03   // 3x uint16 ADC mV (U,V,W) + uint16 pad
#define ORBIT_RPT_STATUS       0x04   // float32 Uq + float32 shaft_vel (rad/s)
#define ORBIT_RPT_LIMITS       0x05   // float32 soft_min + float32 soft_max (deg)
#define ORBIT_RPT_NODE         0x06   // uint8 can_node_id + 7x pad (for bus scan / UI)

// Tuning parameter indices (used by both serial "SP <idx> <val>" and CAN 0x146).
#define ORBIT_PARAM_VEL_P     0
#define ORBIT_PARAM_VEL_I     1
#define ORBIT_PARAM_VEL_D     2
#define ORBIT_PARAM_VEL_RAMP  3
#define ORBIT_PARAM_ANGLE_P   4
#define ORBIT_PARAM_LPF_TF    5
#define ORBIT_PARAM_EKF_EN    6   // stored for protocol compat; ignored (no MT6835 EKF)
#define ORBIT_PARAM_EKF_QA    7
#define ORBIT_PARAM_EKF_QV    8
#define ORBIT_PARAM_EKF_R     9
#define ORBIT_PARAM_TRQ_P     10
#define ORBIT_PARAM_TRQ_I     11
#define ORBIT_PARAM_TRQ_D     12
#define ORBIT_PARAM_TRQ_LPF   13
#define ORBIT_PARAM_SLEW      14   // angle command cruise slew (deg/s); 0 = instant
#define ORBIT_PARAM_ACCEL     15   // angle profile accel (deg/s^2); 0 = step to slew
#define ORBIT_PARAM_DECEL     16   // angle profile decel (deg/s^2); 0 = use accel
#define ORBIT_PARAM_SOFT_MIN  17   // soft limit min (deg), typically negative
#define ORBIT_PARAM_SOFT_MAX  18   // soft limit max (deg), typically positive
#define ORBIT_PARAM_COUNT     19

#define ORBIT_DEF_VEL_P     0.026f
#define ORBIT_DEF_VEL_I     0.30f
#define ORBIT_DEF_VEL_D     0.0012f
#define ORBIT_DEF_VEL_RAMP  70.0f
#define ORBIT_DEF_ANGLE_P   50.0f
#define ORBIT_DEF_LPF_TF    0.05f
// EKF OFF by default — constant-velocity KF lag destabilizes the angle loop
// on halls (same finding as OrbitDrive3.0). Enable via SP 6 1 after soft PID.
#define ORBIT_DEF_EKF_EN    0
#define ORBIT_DEF_EKF_QA    0.001f
#define ORBIT_DEF_EKF_QV    120.0f  // if enabled: high Qv = less lag
#define ORBIT_DEF_EKF_R     0.012f  // if enabled: trust halls less
#define ORBIT_DEF_TRQ_P     4.0f
#define ORBIT_DEF_TRQ_I     10.0f
#define ORBIT_DEF_TRQ_D     0.30f
#define ORBIT_DEF_TRQ_LPF   0.005f
#define ORBIT_DEF_SLEW      360.0f   // deg/s cruise
#define ORBIT_DEF_ACCEL     1000.0f  // deg/s^2
#define ORBIT_DEF_DECEL     1000.0f  // deg/s^2
#define ORBIT_DEF_SOFT_MIN  ORBIT_ANGLE_MIN_DEG
#define ORBIT_DEF_SOFT_MAX  ORBIT_ANGLE_MAX_DEG
#define ORBIT_SLEW_MAX      1500.0f  // deg/s hard max
#define ORBIT_ACCEL_MAX     5000.0f  // deg/s^2 hard max
#define ORBIT_DEF_CAN_NODE  1        // pitch/tilt node

#ifndef ORBIT_GW_PAN_NODE
#define ORBIT_GW_PAN_NODE   0
#endif
#ifndef ORBIT_GW_TILT_NODE
#define ORBIT_GW_TILT_NODE  1
#endif

// Persisted configuration (flash-emulated EEPROM) ------------------------------
// Distinct magic from OrbitDrive3.0 (MT6835) so configs are not mixed.
#define ORBIT_CFG_MAGIC   0x0B17C103UL
#define ORBIT_CFG_VERSION 23  // soft window -135/+135 for the pendulum sweep

struct OrbitConfig {
  uint32_t magic;                // ORBIT_CFG_MAGIC when valid
  uint16_t version;              // ORBIT_CFG_VERSION
  uint8_t  calib_valid;          // 1 = stored electrical calibration is usable
  int8_t   sensor_dir;           // +1 = CW, -1 = CCW
  uint8_t  can_node_id;          // CAN node id (0..15)
  float    zero_electric_angle;  // rad — FOC electrical zero from calibration
  float    mech_zero_offset;     // rad — shaft angle that maps to 0 deg (overwritten by boot home)

  // Control tuning (PID + filters) --------------------------------------------
  float    vel_p;
  float    vel_i;
  float    vel_d;
  float    vel_ramp;
  float    angle_p;
  float    lpf_tf;

  // Hall angle EKF (2-state KF on continuous shaft angle) -------------------
  uint8_t  ekf_enabled;
  float    ekf_q_angle;
  float    ekf_q_vel;
  float    ekf_r_meas;

  // Torque (current) loop -----------------------------------------------------
  float    trq_p;
  float    trq_i;
  float    trq_d;
  float    trq_lpf;

  // Angle command profile: cruise slew + accel/decel (deg, deg/s, deg/s^2).
  // slew_rate 0 = snap instantly. accel/decel 0 = use step (legacy slew-only).
  float    slew_rate;
  float    accel_rate;
  float    decel_rate;

  // Soft angle window (deg). Clamped to hard-span after boot home.
  float    soft_min_deg;
  float    soft_max_deg;
};
