@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
set MPLBACKEND=Agg
set OPENBLAS_NUM_THREADS=1
if not exist ".venv\Scripts\python.exe" (
    echo Run setup_windows.bat first.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" -c "import pytest" >nul 2>nul
if errorlevel 1 (
    echo Install test dependencies first:
    echo .venv\Scripts\python.exe -m pip install -r requirements-dev.txt
    pause
    exit /b 1
)
".venv\Scripts\python.exe" -m pytest -q
set CODE=%errorlevel%
pause
exit /b %CODE%
