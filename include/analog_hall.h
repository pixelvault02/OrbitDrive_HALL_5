#pragma once

#include <stdint.h>

// Analog hall sensing — Clarke → atan2 electrical unwrap → mechanical shaft.
// Ported from OrbitDrive_HALL_2 for GenericSensor callback use.

#ifndef HALL_POLE_PAIRS
#define HALL_POLE_PAIRS 7
#endif

/** Configure hall ADC pins (call once in setup). */
void hallInit();

/**
 * Mechanical shaft angle in [0, 2π) for SimpleFOC GenericSensor.
 * Updates internal electrical unwrap state.
 */
float readHallShaftAngle();

/** Latest filtered electrical angle (rad, wrapped [0, 2π)). */
float hallElRad();

// --- Offset / amplitude calibration (optional peak collect) -----------------
void hallCalResetPeaks();
void hallCalSetCollect(bool on);
bool hallCalCollecting();
void hallCalFinalize();
bool hallCalOk();

// --- αβ ellipse correction (optional collect during spin) ------------------
void hallEllReset();
void hallEllSetCollect(bool on);
bool hallEllCollecting();
void hallEllFinalize();
bool hallEllOk();

/** Raw clarke transform of current filtered hall ADCs (for diagnostics). */
void hallToClarke(float *alpha, float *beta);

/** Filtered hall ADC counts (0..4095) for U,V,W — for UI telemetry. */
void hallAdcFiltered(float out[3]);
