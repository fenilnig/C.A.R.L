// C.A.R.L. MCU configuration. Pin map and limits live here so wiring changes touch one file.
// The pin numbers are ASSUMED (no wiring existed yet): wire the car to match, or edit them here.
// See docs/WIRING_AND_CALIBRATION.md for the wiring table and the bench checks.
#pragma once

// ---- BTS7960 drivers (rear wheels). RPWM = forward, LPWM = reverse. All 4 EN pins (R_EN, L_EN x 2) go to PIN_MOTOR_EN.
#define PIN_MOTOR_EN   7     // LOW at reset, so the drivers are off until the first command arrives
#define PIN_L_RPWM     5
#define PIN_L_LPWM     6
#define PIN_R_RPWM     9
#define PIN_R_LPWM     10
#define L_INVERT       0     // set 1 if the left wheel turns backwards on the bench (or swap the motor wires)
#define R_INVERT       0     // same for the right wheel

// ---- servos (use the 6 V UBEC rail, never the 5 V logic rail; common ground with the UNO Q)
#define PIN_STEER      3     // steering, one servo on the Ackermann linkage
#define PIN_PAN        2
#define PIN_TILT       8
#define SERVO_US_MIN   600   // pulse at 0 deg (calibrate on the bench, MG995/MG996R/MG90S differ)
#define SERVO_US_MAX   2400  // pulse at 180 deg
#define STEER_MIN_DD   600   // steering servo limits in tenths of a degree (90 +/- 30 = the 30 deg mechanical limit)
#define STEER_MAX_DD   1200
#define PAN_MIN_DD     0     // pan servo full 0..180
#define PAN_MAX_DD     1800
#define TILT_MIN_DD    600   // tilt: 60..135 = -30..+45 deg around level (centre 90)
#define TILT_MAX_DD    1350

// ---- other pins
#define PIN_STATUS_LED 4     // solid = linked, slow blink = waiting for the phone/Linux side, fast blink = fault
#define PIN_BATT       A0    // 3S pack through 22k (top) / 6.8k (bottom)
#define BATT_R_TOP     22000.0f
#define BATT_R_BOTTOM  6800.0f
#define ADC_VREF       3.3f  // UNO Q logic is 3.3 V (Linux side divider default of 5 V does not apply here)

// ---- safety (independent of the Linux side)
#define WATCHDOG_MS        300      // no ctl() for this long: wheels brake, steering/gimbal hold
#define BATT_CUTOFF_MV     10000    // 3S, 3.33 V per cell; the Linux side stops a bit earlier at 10.5 V
#define BATT_RESUME_MV     10800    // hysteresis so it does not chatter at the threshold
#define BENCH_USB_ONLY     0        // 1 = ignore the battery divider (board on USB only, nothing on the pack)
#define TOF_HARD_STOP_MM   80       // forward motion is cut below this, whatever the phone says
#define RAMP_PM_PER_LOOP   40       // wheel duty rises at most this per 10 ms loop (0 to full in 0.25 s); braking is instant

// ---- ToF sensor on the I2C bus (Wire, A4/A5)
#define TOF_TIMING_US      33000
#define TOF_RETRY_MS       2000     // keep trying to find a sensor plugged in after boot

#define FW_VERSION "carl-mcu 1.0"

// flag bits returned by ctl(), must match software/mcu_link.py
#define FLAG_WATCHDOG   1
#define FLAG_BATT_CUT   2
#define FLAG_TOF_FAULT  4
#define FLAG_TOF_STOP   8
