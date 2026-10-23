"""Actual Tk/Matplotlib widget smoke test. Linux: xvfb-run -a python tools/gui_smoke.py"""
from pathlib import Path
import sys,os,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
os.environ['OPENBLAS_NUM_THREADS']='1'
import tkinter as tk
from PIL import ImageGrab
from ccpp_mowing.gui import App
from ccpp_mowing.motion import evaluate_events
from ccpp_mowing.pipeline import write_json
import ccpp_mowing.gui as gm

rows=[];out=ROOT/'docs'/'qa';out.mkdir(exist_ok=True)
root=tk.Tk();app=App(root)

def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        root.update();time.sleep(.01)

def wait_done(timeout=60):
    deadline=time.monotonic()+timeout
    while app.process is not None and time.monotonic()<deadline:pump(.05)
    if app.process is not None:raise AssertionError('GUI worker timeout')
    pump(.2)

def check(name,condition):
    if not condition:raise AssertionError(name)
    rows.append({'test':name,'passed':True})

try:
    pump(.3)
    app.vars['grid_type'].set('triangle');app.vars['cell_area'].set('.6');app.vars['width'].set('12');app.vars['height'].set('10')
    app.vars['solver'].set('mst_euler');app.vars['planner'].set('prm');app.vars['shape'].set('notched')
    app.run_button.invoke();check('real_button_launches_owned_subprocess',app.process is not None)
    proc=app.process;check('run_disabled_while_busy',str(app.run_button['state'])=='disabled')
    wait_done();check('worker_exits_and_result_loads',proc.returncode==0 and app.result is not None)
    check('GUI_selection_reaches_pipeline',app.result.config.grid_type=='triangle' and app.result.config.solver=='mst_euler' and app.result.config.planner=='prm')
    check('result_is_independently_validated',app.result.validation['valid'])
    check('run_reenabled',str(app.run_button['state'])=='normal')
    app.first_turn();pump(.2);check('first_turn_selects_slow_follow_view',app.follow.get() and app.speed.get()=='1')
    t0=app.time_value;app.play_button.invoke();pump(.35);check('animation_advances',app.time_value>t0)
    app.play_button.invoke();paused=app.time_value;pump(.2);check('pause_clears_animation_callbacks',abs(app.time_value-paused)<1e-9)
    duration=app.result.events[-1]['t1'];app.slider.set(duration*.5);pump(.1)
    expected=evaluate_events(app.result.events,[duration*.5],app.result.robot)[0]
    xy=app.body.get_xy()[:4].mean(axis=0)
    check('body_position_matches_same_analytic_trajectory',max(abs(xy-expected[1:3]))<1e-7)
    app.follow.set(False);app.redraw_background();pump(.2)
    check('full_map_view_restored',abs(app.ax.get_xlim()[0]-app.full_limits[0][0])<1e-8)
    app.notebook.select(1);pump(.2);check('speed_tab_has_plotted_trajectory',len(app.speed_ax.lines)>=2)
    app.notebook.select(2);pump(.2);check('audit_log_shows_true_planner', 'prm' in app.report.get('1.0','end'))
    app.notebook.select(0);app.reset_animation();pump(.3)
    ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(ROOT/'docs'/'assets'/'gui.png')
    check('real_GUI_screenshot_saved',(ROOT/'docs'/'assets'/'gui.png').stat().st_size>10000)
    figure=out/'gui_export.png';gm.filedialog.asksaveasfilename=lambda **kw:str(figure)
    app.save_figure();check('figure_export_button_works',figure.exists())
    notices=[];gm.messagebox.showerror=lambda title,msg:notices.append((title,msg))
    app.vars['cell_area'].set('bad');app.run_button.invoke();check('bad_input_rejected_without_worker',app.process is None and bool(notices))
    app.vars['cell_area'].set('.12');app.vars['width'].set('30');app.vars['height'].set('24')
    app.run_button.invoke();p=app.process;folder=app.run_dir;app.cancel_button.invoke();wait_done()
    check('cancellation_terminates_only_owned_worker',p.poll() is not None)
    check('cancelled_output_never_published_as_success',json.loads((folder/'status.json').read_text())['status']=='cancelled')
    app.run_button.invoke();p=app.process;app.close()
    check('closing_reaps_child_process',p.poll() is not None)
except Exception:
    import traceback
    rows.append({'passed':False,'error':traceback.format_exc()})
    try:app.close()
    except Exception:pass
write_json(out/'gui_tests.json',{'platform':sys.platform,'tests':rows,'passed':sum(r['passed'] for r in rows),'total':len(rows)})
print(json.dumps(rows,indent=2))
raise SystemExit(0 if all(r['passed'] for r in rows) else 1)
