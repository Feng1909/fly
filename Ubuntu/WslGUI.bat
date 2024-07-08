@echo off

REM Set the path of the RflySim tools
if not defined PSP_PATH (
    SET PSP_PATH=C:\Softwares\PX4PSP
    SET PSP_PATH_LINUX=/mnt/c/Softwares/PX4PSP
)

cd /d %PSP_PATH%\VcXsrv
Xlaunch.exe -run config.xlaunch

choice /t 1 /d y /n >nul

wsl -d RflySim-20.04 ~/StartUI.sh
choice /t 1 /d y /n >nul
tasklist|find /i "vcxsrv.exe" && taskkill /f /IM "vcxsrv.exe"
wsl --shutdown
