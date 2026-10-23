"""Compatibility entry: python tools/render_clip.py FOLDER --out FILE.gif."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ccpp_mowing.cli import main
if __name__=='__main__':raise SystemExit(main(['render',*sys.argv[1:]]))
