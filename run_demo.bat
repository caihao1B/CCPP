@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
if not exist ".venv\Scripts\python.exe" (
 echo Run setup_windows.bat first.
 pause
 exit /b 1
)
echo This recomputes the recommended demo into outputs\demo08.
echo Existing generated demo08 files will be replaced.
".venv\Scripts\python.exe" -m ccpp_mowing plan --config configs/demo_08.json --map maps/demo_08.geojson --robot configs/robot_08.json --out outputs/demo08 --overwrite
set CODE=%errorlevel%
echo Open outputs\demo08 in the GUI when validation passes.
pause
exit /b %CODE%
