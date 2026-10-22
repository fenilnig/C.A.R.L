@echo off
rem Push the controller to the UNO Q over USB, run the tests there, install/restart the boot service, show the address.
rem Usage: double-click, then read H:\carl\docs\deploy_log.txt
setlocal
set LOG=H:\carl\docs\deploy_log.txt
set ADB=
for /d %%D in ("%LOCALAPPDATA%\Arduino15\packages\arduino\tools\adb\*") do set ADB=%%D\adb.exe
echo ADB=%ADB% > "%LOG%"
"%ADB%" devices >> "%LOG%" 2>&1
"%ADB%" shell "mkdir -p /home/arduino/carl" >> "%LOG%" 2>&1
"%ADB%" push H:\carl\software\. /home/arduino/carl/ >> "%LOG%" 2>&1
echo --- tests >> "%LOG%"
"%ADB%" shell "cd /home/arduino/carl && python3 tests/test_core.py | tail -2 && python3 tests/test_mcu_link.py | tail -2 && python3 tests/test_server.py | tail -2" >> "%LOG%" 2>&1
echo --- service >> "%LOG%"
"%ADB%" shell "cp /home/arduino/carl/carl.service /etc/systemd/system/carl.service && systemctl daemon-reload && systemctl enable carl.service && systemctl restart carl.service && sleep 2 && systemctl is-active carl.service" >> "%LOG%" 2>&1
echo --- address >> "%LOG%"
"%ADB%" shell "hostname -I; nmcli -t -f ACTIVE,SSID dev wifi | head -5; curl -s localhost:8000/status" >> "%LOG%" 2>&1
type "%LOG%"
pause
