#include "analog_hall.h"
#include "board_pins.h"
#include <Arduino.h>
#include <math.h>
#include <SimpleFOC.h>

#define HALL_ADC_TF    0.0035f   // slightly heavier ADC LPF for quieter angle
#define HALL_EL_TF     0.0025f
#define HALL_ADC_AVG   6
#define HALL_AMP_MIN   0.08f

static float cal_off[3] = {2048.0f, 2048.0f, 2048.0f};
static float cal_gain[3] = {1.0f / 350.0f, 1.0f / 350.0f, 1.0f / 350.0f};
static float cal_min[3] = {4095.0f, 4095.0f, 4095.0f};
static float cal_max[3] = {0.0f, 0.0f, 0.0f};
static bool cal_ok = false;
static bool cal_collect = false;

static bool ell_ok = false;
static bool ell_collect = false;
static float ell_off_a = 0.0f, ell_off_b = 0.0f;
static float ell_scale_a = 1.0f, ell_scale_b = 1.0f;
static float ell_k = 0.0f;
static uint32_t ell_n = 0;
static double ell_sa = 0, ell_sb = 0, ell_sa2 = 0, ell_sb2 = 0, ell_sab = 0;

static float hall_el_fast = 0.0f;
static float pos_el = 0.0f;
static float hall_adc_f[3] = {2048.0f, 2048.0f, 2048.0f};
static uint32_t hall_us = 0;

static uint16_t readAdcAvg(uint32_t pin, uint8_t n = 8) {
  (void)analogRead(pin);
  if (n < 1) n = 1;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < n; i++) sum += analogRead(pin);
  return (uint16_t)(sum / n);
}

static float wrapTwoPi(float a) {
  while (a < 0.0f) a += _2PI;
  while (a >= _2PI) a -= _2PI;
  return a;
}

static float wrapPi(float a) {
  while (a > PI) a -= _2PI;
  while (a < -PI) a += _2PI;
  return a;
}

static float lpf1(float y, float x, float tf, float dt) {
  float a = dt / (tf + dt);
  if (a > 1.0f) a = 1.0f;
  return y + a * (x - y);
}

void hallInit() {
  analogReadResolution(12);
  pinMode(PIN_HALL_U, INPUT_ANALOG);
  pinMode(PIN_HALL_V, INPUT_ANALOG);
  pinMode(PIN_HALL_W, INPUT_ANALOG);
}

float hallElRad() { return hall_el_fast; }

void hallEllReset() {
  ell_ok = false;
  ell_n = 0;
  ell_sa = ell_sb = ell_sa2 = ell_sb2 = ell_sab = 0;
  ell_off_a = ell_off_b = 0.0f;
  ell_scale_a = ell_scale_b = 1.0f;
  ell_k = 0.0f;
}

void hallEllSetCollect(bool on) { ell_collect = on; }
bool hallEllCollecting() { return ell_collect; }
bool hallEllOk() { return ell_ok; }

static void ellObserve(float alpha, float beta) {
  if (!ell_collect) return;
  if (!isfinite(alpha) || !isfinite(beta)) return;
  float r2 = alpha * alpha + beta * beta;
  if (r2 < 1e-6f) return;
  ell_n++;
  ell_sa += alpha;
  ell_sb += beta;
  ell_sa2 += (double)alpha * alpha;
  ell_sb2 += (double)beta * beta;
  ell_sab += (double)alpha * beta;
}

void hallEllFinalize() {
  ell_ok = false;
  if (ell_n < 80) return;
  double inv = 1.0 / (double)ell_n;
  double ma = ell_sa * inv;
  double mb = ell_sb * inv;
  double caa = ell_sa2 * inv - ma * ma;
  double cbb = ell_sb2 * inv - mb * mb;
  double cab = ell_sab * inv - ma * mb;
  if (caa < 1e-6 || cbb < 1e-6) return;
  float sa = (float)sqrt(caa);
  float sb = (float)sqrt(cbb);
  float k = (float)(cab / (sa * sb));
  if (k > 0.95f) k = 0.95f;
  if (k < -0.95f) k = -0.95f;
  ell_off_a = (float)ma;
  ell_off_b = (float)mb;
  ell_scale_a = 1.0f / sa;
  ell_scale_b = 1.0f / sb;
  ell_k = k;
  ell_ok = true;
}

void hallCalResetPeaks() {
  for (uint8_t i = 0; i < 3; i++) {
    cal_min[i] = 4095.0f;
    cal_max[i] = 0.0f;
  }
  hallEllReset();
}

void hallCalSetCollect(bool on) { cal_collect = on; }
bool hallCalCollecting() { return cal_collect; }
bool hallCalOk() { return cal_ok; }

static void calObserve(float u, float v, float w) {
  if (!cal_collect) return;
  float x[3] = {u, v, w};
  for (uint8_t i = 0; i < 3; i++) {
    if (x[i] < cal_min[i]) cal_min[i] = x[i];
    if (x[i] > cal_max[i]) cal_max[i] = x[i];
  }
}

void hallCalFinalize() {
  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    float amp = 0.5f * (cal_max[i] - cal_min[i]);
    if (amp < 80.0f) ok = false;
    cal_off[i] = 0.5f * (cal_max[i] + cal_min[i]);
    cal_gain[i] = (amp > 80.0f) ? (1.0f / amp) : (1.0f / 350.0f);
  }
  cal_ok = ok;
  Serial.print(F("HALL phase cal="));
  Serial.println(cal_ok ? F("ok") : F("weak"));
}

static void applyEllipse(float *alpha, float *beta) {
  if (!ell_ok) return;
  float a = (*alpha - ell_off_a) * ell_scale_a;
  float b = (*beta - ell_off_b) * ell_scale_b;
  float den = sqrtf(fmaxf(1e-4f, 1.0f - ell_k * ell_k));
  b = (b - ell_k * a) / den;
  *alpha = a;
  *beta = b;
}

void hallToClarke(float *alpha, float *beta) {
  float u, v, w;
  if (cal_ok) {
    u = (hall_adc_f[0] - cal_off[0]) * cal_gain[0];
    v = (hall_adc_f[1] - cal_off[1]) * cal_gain[1];
    w = (hall_adc_f[2] - cal_off[2]) * cal_gain[2];
  } else {
    u = hall_adc_f[0];
    v = hall_adc_f[1];
    w = hall_adc_f[2];
  }
  float mid = (u + v + w) / 3.0f;
  u -= mid;
  v -= mid;
  w -= mid;
  float n = sqrtf(u * u + v * v + w * w);
  if (n > 1e-3f) {
    u /= n;
    v /= n;
    w /= n;
  }
  *alpha = u;
  *beta = (v - w) * 0.57735026919f;
}

float readHallShaftAngle() {
  static float filt = 0.0f;
  static bool inited = false;

  uint32_t us = micros();
  float dt = (hall_us == 0) ? 0.0005f : (float)(us - hall_us) * 1e-6f;
  hall_us = us;
  if (dt < 0.00005f) dt = 0.00005f;
  if (dt > 0.004f) dt = 0.004f;

  float ur = (float)readAdcAvg(PIN_HALL_U, HALL_ADC_AVG);
  float vr = (float)readAdcAvg(PIN_HALL_V, HALL_ADC_AVG);
  float wr = (float)readAdcAvg(PIN_HALL_W, HALL_ADC_AVG);
  calObserve(ur, vr, wr);

  if (!inited) {
    hall_adc_f[0] = ur;
    hall_adc_f[1] = vr;
    hall_adc_f[2] = wr;
  } else {
    hall_adc_f[0] = lpf1(hall_adc_f[0], ur, HALL_ADC_TF, dt);
    hall_adc_f[1] = lpf1(hall_adc_f[1], vr, HALL_ADC_TF, dt);
    hall_adc_f[2] = lpf1(hall_adc_f[2], wr, HALL_ADC_TF, dt);
  }

  float alpha, beta;
  hallToClarke(&alpha, &beta);
  if (ell_collect) ellObserve(alpha, beta);
  applyEllipse(&alpha, &beta);

  float raw = wrapTwoPi(atan2f(beta, alpha));
  float amp2 = alpha * alpha + beta * beta;

  if (amp2 < HALL_AMP_MIN * HALL_AMP_MIN) {
    return wrapTwoPi(pos_el / (float)HALL_POLE_PAIRS);
  }

  if (!inited) {
    filt = raw;
    hall_el_fast = raw;
    pos_el = raw;
    inited = true;
    return wrapTwoPi(pos_el / (float)HALL_POLE_PAIRS);
  }

  float d = wrapPi(raw - filt);
  if (fabsf(d) > 2.8f) {
    return wrapTwoPi(pos_el / (float)HALL_POLE_PAIRS);
  }
  float a_el = dt / (HALL_EL_TF + dt);
  if (a_el > 1.0f) a_el = 1.0f;
  filt = wrapTwoPi(filt + a_el * d);
  hall_el_fast = filt;

  float du = wrapPi(filt - wrapTwoPi(pos_el));
  float max_du = 1.2f;
  if (du > max_du) du = max_du;
  if (du < -max_du) du = -max_du;
  pos_el += du;
  return wrapTwoPi(pos_el / (float)HALL_POLE_PAIRS);
}

void hallAdcFiltered(float out[3]) {
  out[0] = hall_adc_f[0];
  out[1] = hall_adc_f[1];
  out[2] = hall_adc_f[2];
}
