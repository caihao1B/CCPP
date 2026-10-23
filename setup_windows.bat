@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
set PIP_DISABLE_PIP_VERSION_CHECK=1
if exist ".venv\Scripts\python.exe" goto install
py -3.13 -c "import sys; assert sys.version_info >= (3,11)" >nul 2>nul
if not errorlevel 1 (
    py -3.13 -m venv .venv
    goto created
)
py -3.12 -c "import sys; assert sys.version_info >= (3,11)" >nul 2>nul
if not errorlevel 1 (
    py -3.12 -m venv .venv
    goto created
)
python -c "import sys; assert sys.version_info >= (3,11)" >nul 2>nul
if errorlevel 1 (
    echo Python 3.11 or newer was not found. Install 64-bit Python 3.12/3.13 with Tcl/Tk.
    goto failed
)
python -m venv .venv
:created
if not exist ".venv\Scripts\python.exe" goto failed
:install
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto failed
".venv\Scripts\python.exe" -c "import sys, tkinter, numpy, scipy, shapely, networkx, matplotlib, PIL; from ccpp_mowing import __version__; print('CCPP Mowing Planner',__version__); print(sys.executable)"
if errorlevel 1 goto failed
echo.
echo Setup complete. Run run_gui.bat.
echo For development/tests: .venv\Scripts\python.exe -m pip install -r requirements-dev.txt
pause
exit /b 0
:failed
echo.
echo Setup failed. Read the error above. Do not reuse an environment copied from another computer.
echo Network errors: retry after fixing the network, or install wheels from a trusted local source.
pause
exit /b 1
