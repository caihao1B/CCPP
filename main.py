"""VS Code friendly entry point. Plain `python main.py` runs the default experiment.

For another algorithm: python main.py --grid triangle --intra mst_euler --planner prm
Use app.py for the native GUI. No ROS or external simulation engine is needed.
"""
from pathlib import Path
import sys
from ccpp_mowing.cli import main

if __name__=='__main__':
    args=sys.argv[1:]
    if not args:args=['--out',str(Path(__file__).resolve().parent/'outputs'/'latest'),'--overwrite']
    raise SystemExit(main(['plan',*args]))
