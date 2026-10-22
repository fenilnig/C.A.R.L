@echo off
set LOG=H:\carl\docs\start_server_log.txt
set ADB=
for /d %%D in ("%LOCALAPPDATA%\Arduino15\packages\arduino\tools\adb\*") do set ADB=%%D\adb.exe
"%ADB%" push H:\carl\software\. /home/arduino/carl/ > "%LOG%" 2>&1
"%ADB%" shell "pkill -f '[s]erver\.py'" >> "%LOG%" 2>&1
"%ADB%" shell "cd /home/arduino/carl && setsid nohup python3 server.py > server.log 2>&1 < /dev/null &" >> "%LOG%" 2>&1
timeout /t 4 >nul
"%ADB%" shell "pgrep -af '[s]erver\.py'; curl -s localhost:8000/status; echo; ip -4 -o addr show | cut -d' ' -f2,7" >> "%LOG%" 2>&1
echo --- done >> "%LOG%"
