@echo off
echo python3 main.py
start wsl -d RflySim-20.04 -e bash -lic "python3 main.py"
choice /t 20 /d y /n >nul
echo ./StartSLAM.sh
start wsl -d RflySim-20.04 -e bash -lic "./StartSLAM.sh"
choice /t 5 /d y /n >nul
echo python3 testRflyRosCtrl.py
start wsl -d RflySim-20.04 -e bash -lic "python3 testRflyRosCtrl.py"
