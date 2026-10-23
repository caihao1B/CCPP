"""Capture the actual GUI after loading a verified fine-grid example."""
from pathlib import Path
import sys,time,tkinter as tk
from PIL import ImageGrab
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ccpp_mowing.gui import App
if __name__=='__main__':
    root=tk.Tk();app=App(root);app.load(ROOT/'examples/square_default')
    app.vars['grid_type'].set('square');app.vars['solver'].set('serpentine');app.vars['planner'].set('astar')
    for _ in range(8):root.update();time.sleep(.06)
    ImageGrab.grab().crop((root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())).save(ROOT/'docs/assets/gui.png')
    app.first_turn();app.draw_robot()
    for _ in range(8):root.update();time.sleep(.06)
    ImageGrab.grab().crop((root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())).save(ROOT/'docs/assets/gui_turn.png')
    app.close();print('Actual GUI captures saved.')
