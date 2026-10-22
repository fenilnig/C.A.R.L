# C.A.R.L. - Camera-Assisting Robotic Locomotive
Pinnacle project, Self Driving Cars (CIA 1 and CIA 2). Due 30 Oct 2026.

A small 4-wheel printed rover that carries a Sony a6700 on a pan/tilt gimbal, driven from a phone over Wi-Fi.

## Folders
- blueprint/  3D blueprint page (open in a browser). Live copy: https://claude.ai/artifact/7KdVvVXjZYJoFxdoWBCyMC
- software/   phone controller (drive, steer, camera pan/tilt, e-stop). Runs on the UNO Q as the `carl` boot service,
              open http://<ip>:8000 on the phone. Deploy: double-click `deploy_unoq.bat` (pushes, tests, restarts the service).
              Tests: `python3 tests/test_core.py`, `tests/test_mcu_link.py`, `tests/test_server.py`
- firmware/   carl_mcu: the STM32 sketch (drivers, servos, ToF, battery, own safety net). Upload with the Arduino IDE
              (board: Arduino UNO Q). Needs the `Servo` and `VL53L0X` libraries.
- prints/     STL files (shaft fit-test plate so far)
- cad/        precise CAD of all 29 printed parts (STL, STEP, parametric Python); docs/PRINT_PLAN.md says which parts to print in which order
- docs/       notes, shopping list, WIRING_AND_CALIBRATION.md (pin map, first power-up checklist, calibration)

## Key numbers (Rev G)
- Wheelbase 260 mm, track 208 mm, wheels 100 x 40 mm
- Demo speed cap 1 m/s, steering inner-wheel cap 20 deg
- Motor: shaft 6.6 mm, gearbox 38.4 mm, can 31.8 mm, M3 holes 29.8 mm apart
- Safety: deadman 0.3 s, ToF auto-brake, low-battery stop, emergency stop

## Schedule
21-22 Oct pay carts, shop trip, first print batch | 23-25 bench tests | 26-27 assembly and final gimbal
28 Review 1 and 2 plus backup video | 29 mount a6700, tune, test | 30 submit
