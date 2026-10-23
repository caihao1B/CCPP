#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONUTF8=1 PYTHONIOENCODING=utf-8
python3 -c 'import sys; assert sys.version_info >= (3,11), "Python >=3.11 required"'
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -c 'import tkinter; from ccpp_mowing import __version__; print(__version__)'
echo 'Ready: bash run_gui.sh. Linux needs Tcl/Tk and a graphical display.'
