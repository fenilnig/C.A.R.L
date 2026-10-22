// C.A.R.L. MCU firmware for the Arduino UNO Q (STM32U585 side).
// The Linux side (software/server.py) calls ctl() ~50 times a second through arduino-router.
// This sketch only clamps and drives pins; the driving maths and most safety logic live in Python.
// It still protects the car on its own: watchdog, battery cutoff and a hard ToF stop.
#include <Arduino_RouterBridge.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "config.h"

Servo steerServo, panServo, tiltServo;
VL53L0X tof;

// written by the RPC thread, read by loop(); single 32-bit writes, a mixed frame lasts at most one 10 ms loop
volatile int tgtL = 0, tgtR = 0;
volatile int tgtSteer = 900, tgtPan = 900, tgtTilt = 900;
volatile uint32_t lastCtlMs = 0;
volatile bool haveCtl = false;
volatile int tofMm = -1;
volatile int battMv = 0;
volatile int flags = 0;

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int ddToUs(int dd) { return SERVO_US_MIN + (long)(SERVO_US_MAX - SERVO_US_MIN) * clampi(dd, 0, 1800) / 1800; }

static void driveMotor(int rpwm, int lpwm, int pm, bool invert) {
  if (invert) pm = -pm;
  const int duty = (int)((long)abs(pm) * 1023 / 1000);
  if (pm > 0)      { analogWrite(lpwm, 0); analogWrite(rpwm, duty); }
  else if (pm < 0) { analogWrite(rpwm, 0); analogWrite(lpwm, duty); }
  else             { analogWrite(rpwm, 0); analogWrite(lpwm, 0); }   // both low sides on = brake
}

// RPC: called from the bridge thread
MsgPack::arr_t<int32_t> ctl(int l_pm, int r_pm, int steer_dd, int pan_dd, int tilt_dd) {
  tgtL = clampi(l_pm, -1000, 1000);
  tgtR = clampi(r_pm, -1000, 1000);
  tgtSteer = clampi(steer_dd, STEER_MIN_DD, STEER_MAX_DD);
  tgtPan = clampi(pan_dd, PAN_MIN_DD, PAN_MAX_DD);
  tgtTilt = clampi(tilt_dd, TILT_MIN_DD, TILT_MAX_DD);
  lastCtlMs = millis();
  haveCtl = true;
  const int32_t tof = tofMm, batt = battMv, fl = flags;   // copies: volatile values cannot go into the vector
  MsgPack::arr_t<int32_t> out;
  out.push_back(tof);
  out.push_back(batt);
  out.push_back(fl);
  return out;
}

MsgPack::str_t ping() { return MsgPack::str_t(FW_VERSION); }

static bool tofOk = false;
static uint32_t nextTofTry = 0;

static void tofBegin() {
  tof.setTimeout(100);
  tofOk = tof.init();
  if (tofOk) {
    tof.setMeasurementTimingBudget(TOF_TIMING_US);
    tof.startContinuous();
  }
}

static void readSensors() {
  // ToF: -1 means "no valid reading" (the Linux side then treats it as an obstacle at 0 m)
  if (!tofOk) {
    tofMm = -1;
    if (millis() > nextTofTry) { nextTofTry = millis() + TOF_RETRY_MS; tofBegin(); }
  } else {
    const uint16_t mm = tof.readRangeContinuousMillimeters();
    tofMm = tof.timeoutOccurred() ? -1 : (int)(mm > 8190 ? 8190 : mm);
    if (tofMm < 0) tofOk = false;      // re-init next round, a loose wire recovers by itself
  }
  const float v = analogRead(PIN_BATT) / 4095.0f * ADC_VREF * (BATT_R_TOP + BATT_R_BOTTOM) / BATT_R_BOTTOM;
  battMv = (int)(v * 1000.0f);
}

static int rampL = 0, rampR = 0;
static bool battCut = false;

static int ramp(int cur, int target) {
  if (abs(target) < abs(cur) || (target > 0) != (cur > 0)) return target;    // braking or reversing: instant
  return clampi(target, cur - RAMP_PM_PER_LOOP, cur + RAMP_PM_PER_LOOP);
}

void setup() {
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, LOW);          // drivers off until the first command
  pinMode(PIN_STATUS_LED, OUTPUT);
  analogWriteResolution(10);
  analogReadResolution(12);
  driveMotor(PIN_L_RPWM, PIN_L_LPWM, 0, false);
  driveMotor(PIN_R_RPWM, PIN_R_LPWM, 0, false);
  steerServo.attach(PIN_STEER);
  panServo.attach(PIN_PAN);
  tiltServo.attach(PIN_TILT);
  steerServo.writeMicroseconds(ddToUs(tgtSteer));
  panServo.writeMicroseconds(ddToUs(tgtPan));
  tiltServo.writeMicroseconds(ddToUs(tgtTilt));
  Wire.begin();
  tofBegin();
  Bridge.begin();
  Bridge.provide("ctl", ctl);
  Bridge.provide("ping", ping);
}

void loop() {
  const uint32_t now = millis();
  readSensors();

  int f = 0;
  const bool stale = !haveCtl || (now - lastCtlMs) > WATCHDOG_MS;
  if (haveCtl && stale) f |= FLAG_WATCHDOG;

#if BENCH_USB_ONLY
  battCut = false;
#else
  if (battMv < BATT_CUTOFF_MV) battCut = true;
  else if (battMv > BATT_RESUME_MV) battCut = false;
#endif
  if (battCut) f |= FLAG_BATT_CUT;
  if (tofMm < 0) f |= FLAG_TOF_FAULT;

  int l = tgtL, r = tgtR;
  if (stale || battCut) { l = 0; r = 0; }
  // forward only: a very close obstacle cuts forward drive, reversing is always allowed
  if (tofMm >= 0 && tofMm < TOF_HARD_STOP_MM) {
    if (l > 0) { l = 0; f |= FLAG_TOF_STOP; }
    if (r > 0) { r = 0; f |= FLAG_TOF_STOP; }
  }
  rampL = ramp(rampL, l);
  rampR = ramp(rampR, r);

  // EN stays low before the first command and during a battery cutoff, otherwise high (duty 0 = brake)
  digitalWrite(PIN_MOTOR_EN, (haveCtl && !battCut) ? HIGH : LOW);
  driveMotor(PIN_L_RPWM, PIN_L_LPWM, rampL, L_INVERT);
  driveMotor(PIN_R_RPWM, PIN_R_LPWM, rampR, R_INVERT);

  steerServo.writeMicroseconds(ddToUs(tgtSteer));
  panServo.writeMicroseconds(ddToUs(tgtPan));
  tiltServo.writeMicroseconds(ddToUs(tgtTilt));

  flags = f;
  // LED: solid = linked, 1 Hz = waiting for the Linux side, 5 Hz = fault
  const bool fault = (f & (FLAG_BATT_CUT | FLAG_TOF_FAULT)) != 0;
  const uint32_t period = fault ? 100 : (haveCtl && !stale ? 0 : 500);
  digitalWrite(PIN_STATUS_LED, period == 0 ? HIGH : ((now / period) & 1));

  delay(10);
}
