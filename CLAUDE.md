# C.A.R.L. project context (for Claude Code)
C.A.R.L. = Camera-Assisting Robotic Locomotive. A 4-wheel printed rover with a Sony a6700 gimbal, driven from a phone web app.
See README.md, docs/shopping_and_status.md, cad/README_CAD.md, docs/WIRING_AND_CALIBRATION.md. Deadline 30 Oct 2026.

## Status (22 Oct 2026)
Done: controller on the UNO Q as systemd service `carl` (starts on boot), phone UI with drive/steer/camera sticks,
Linux-to-MCU link (`software/mcu_link.py`, tested against the real arduino-router), MCU firmware `firmware/carl_mcu`
(compiles for arduino:zephyr:unoq). Python tests pass on the laptop and on the board.
Not done: firmware not uploaded yet (needs the Servo and VL53L0X libraries), board not on Wi-Fi yet, nothing wired,
all pin numbers and calibration constants unverified (see docs/WIRING_AND_CALIBRATION.md).

## How to work here
- UNO Q is on USB (adb serial 2391793461). adb: %LOCALAPPDATA%\Arduino15\packages\arduino\tools\adb\32.0.0\adb.exe
  In Git Bash set MSYS_NO_PATHCONV=1 or /home/... paths get mangled.
- Deploy: `deploy_unoq.bat` (log in docs\deploy_log.txt). Service: `systemctl status carl` on the board.
- Compile firmware: Arduino IDE's arduino-cli, `--fqbn arduino:zephyr:unoq`, sketch folder firmware\carl_mcu.
- Do not guess Wi-Fi names or passwords, ask Fenil. Do not change files in cad\ or blueprint\ unless asked.
- Safety: motors are not connected yet. Keep the ToF requirement on; `CARL_NO_TOF=1` is for bench only.
