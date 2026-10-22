# C.A.R.L. wiring and calibration

The pin map below is an assumption (no wiring existed when the firmware was written). It lives in
`firmware/carl_mcu/config.h`; change it there if you wire differently, then re-upload.

## Architecture
```
phone (browser) --Wi-Fi--> server.py (UNO Q Linux) --msgpack-RPC over arduino-router--> carl_mcu (STM32 MCU) --> pins
```
- `software/dolly_core.py` driving maths and safety filter (deadman 0.3 s, ToF auto-brake, low battery, e-stop)
- `software/mcu_link.py` unit conversion and the RPC client. Calibration constants are at the top of this file
- `firmware/carl_mcu/` pin driver with its own safety net: watchdog 300 ms, battery cutoff 10.0 V, hard ToF stop 80 mm,
  soft-start ramp, drivers disabled until the first command

## Pin map (UNO Q header, UNO layout)
| Function | Pin | Notes |
|---|---|---|
| BTS7960 enable (all four R_EN/L_EN) | D7 | LOW at reset: drivers off |
| Left drive RPWM / LPWM | D5 / D6 | forward / reverse |
| Right drive RPWM / LPWM | D9 / D10 | |
| Steering servo signal | D3 | 6 V UBEC rail, not the 5 V logic rail |
| Pan servo signal | D2 | final build: MG996R |
| Tilt servo signal | D8 | final build: MG996R |
| Status LED | D4 | solid = linked, 1 Hz = waiting for phone, 5 Hz = fault |
| Battery divider | A0 | 22 k to pack+, 6.8 k to GND, midpoint to A0 (12.6 V gives 2.98 V, safe for the 3.3 V ADC) |
| VL53L0X ToF | SDA/SCL (A4/A5) | 3.3 V supply, I2C address 0x29 |
| MPU6050 | SDA/SCL (A4/A5) | wired in the blueprint but not used by the software yet |

Rules: one common ground between the pack, both buck/UBEC rails, the BTS7960 boards and the UNO Q. The pack goes
through the 10 A fuse. The UNO Q is powered from the 5 V buck, never from a servo rail. Motor power goes only to
the BTS7960 B+/B-.

PWM is fixed at 500 Hz by the board package, so the motors will whine slightly. That is normal and harmless.

## First power-up checklist (wheels off the ground, motors disconnected first)
1. Upload the firmware. The status LED blinks at 1 Hz (no phone yet).
2. Start `server.py`, open the phone page. The LED goes solid and the page shows `MCU ok`.
3. Close the phone page or lock the phone: within 0.3 s the LED returns to 1 Hz (watchdog works).
4. Unplug the ToF sensor: LED blinks fast and the page shows ToF 0 cm, forward is blocked, reverse still works.
5. Connect the steering servo with the horn removed. Stick left/right moves it. If left is right, set
   `STEER_SIGN = -1.0` in `mcu_link.py`.
6. Put the horn on with the wheels straight, then trim `STEER_CENTER_DEG` until the wheels are straight.
7. Set `STEER_GAIN` so that the inner wheel reaches 20 degrees at full stick (measure with a protractor or the
   fit sheet). Start at 1.0.
8. Connect the motors with the wheels in the air at low stick. If a wheel runs backwards swap `L_INVERT`/`R_INVERT`
   in `config.h`, or swap its two motor wires.
9. Gimbal: if pan or tilt moves the wrong way, set `PAN_SIGN` / `TILT_SIGN = -1.0` in `mcu_link.py`.
   Set `TILT_CENTER_DEG` so the a6700 is level at stick-centre.
10. Battery: compare the number on the phone with a multimeter. If it is off, adjust `ADC_VREF` in `config.h`.

## Calibration constants (software/mcu_link.py)
| Name | Default | Meaning |
|---|---|---|
| `V_FREE_MPS` | 2.6 | speed at 100 % duty (500 RPM, 100 mm wheel). Open loop. With the 1 m/s demo cap the car uses about 38 % duty |
| `STEER_CENTER_DEG` | 90 | servo angle for straight |
| `STEER_GAIN` | 1.0 | servo degrees per degree of Ackermann angle |
| `STEER_SIGN`, `PAN_SIGN`, `TILT_SIGN` | +1 | flip when a direction is wrong |
| `TILT_CENTER_DEG` | 90 | level a6700 |

Servo pulse range is `SERVO_US_MIN/MAX` (600 to 2400 us) in `config.h`. If a servo buzzes at an end, narrow it.

## Bench-test modes (environment variables for server.py)
- `CARL_MOTORS=stub` run without the MCU (laptop). Default `auto` uses the MCU if arduino-router exists.
- `CARL_NO_TOF=1` allow forward motion with no ToF sensor plugged in (bench only, never on the floor).
- Firmware: `BENCH_USB_ONLY 1` in `config.h` when the board is on USB with no battery connected.

## Not done / needs the real hardware
- Wheel speed is open loop (no encoders), so speed accuracy depends on battery voltage and load.
- The MPU6050 is not read. The ToF sensor looks forward only, so reversing has no obstacle detection.
- All pin numbers, servo ranges and `STEER_GAIN` are unverified until the car is wired.
