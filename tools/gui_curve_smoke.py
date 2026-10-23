"""Real Tk widgets + child process + pose replay checks. Linux: xvfb-run -a."""
from pathlib import Path
import sys,os,json,time
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import tkinter as tk
from PIL import ImageGrab
from ccpp_mowing.gui import App
import ccpp_mowing.gui as gm
from ccpp_mowing.motion import evaluate_events
from ccpp_mowing.animation_export import first_curve
from ccpp_mowing.pipeline import write_json

out=ROOT/'docs/qa';out.mkdir(parents=True,exist_ok=True)
root=tk.Tk();app=App(root);rows=[]

def pump(t=.15):
    end=time.monotonic()+t
    while time.monotonic()<end:root.update();time.sleep(.005)

def check(name,condition):
    assert condition,name
    rows.append({'name':name,'passed':True})

def wait_worker(timeout=80):
    end=time.monotonic()+timeout
    while app.process is not None and time.monotonic()<end:pump(.05)
    check('worker_finishes',app.process is None)

try:
    pump(.4)
    check('motion_models_exposed','motion_model' in app.vars)
    check('curve_methods_exposed','trajectory_method' in app.vars)
    check('original_and_forward_loop_presets',len(app.preset_rows)>=12)
    app.preset_value.set(next(r['name'] for r in app.preset_rows if r['id']=='03'));app.apply_preset()
    check('preset_applies_optimized_bezier',app.vars['trajectory_method'].get()=='bezier_slsqp')
    app.run_button.invoke();proc=app.process
    check('real_GUI_button_launches_pipeline',proc is not None)
    wait_worker();pump(.3)
    check('curved_result_loaded',app.result is not None and app.result.config.trajectory_method=='bezier_slsqp')
    check('curved_result_passes_independent_validation',app.result.validation['valid'])
    check('result_contains_real_Bezier_segments',app.result.metrics['curve_primitive_counts'].get('BEZIER5',0)>0)
    check('four_result_tabs',len(app.notebook.tabs())==4)
    app.first_turn();pump(.2)
    check('first_curve_enables_slow_follow',app.follow.get() and app.speed.get()=='1')
    turn=first_curve(app.result.events);t=turn['t0']+turn['duration']*.53
    app.slider.set(t);pump(.2)
    expected=evaluate_events(app.result.events,[t],app.result.robot)[0]
    check('body_on_analytical_curve',np.max(abs(app.body.get_xy()[:4].mean(axis=0)-expected[1:3]))<1e-8)
    hx,hy=app.heading.get_data();angle=np.arctan2(hy[-1]-hy[0],hx[-1]-hx[0])
    check('heading_tangent_to_curve',abs((angle-expected[3]+np.pi)%(2*np.pi)-np.pi)<1e-8)
    check('moving_while_yaw_changes',expected[4]>0 and abs(expected[5])>1e-3)
    old=app.time_value;app.toggle_play();pump(.25);app.pause()
    check('animation_clock_advances',app.time_value>old)
    old=app.time_value;pump(.15);check('pause_stops_clock',abs(old-app.time_value)<1e-9)
    app.slider.set(t);pump(.2)
    ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(ROOT/'docs/assets/gui_bezier_follow.png')
    app.follow.set(False);app.redraw_background();pump(.2)
    ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(ROOT/'docs/assets/gui_full.png')
    check('curvature_chart_exists',len(app.curve_ax.lines)>=3)
    app.notebook.select(3);pump(.2)
    ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(ROOT/'docs/assets/gui_curvature.png')
    app.notebook.select(0)
    target=out/'gui_figure.png';gm.filedialog.asksaveasfilename=lambda **kw:str(target)
    app.save_figure();check('figure_saves_with_robot',target.exists())
    app.load(ROOT/'examples/demo_04');pump(.2)
    check('saved_config_syncs_to_controls',app.vars['motion_model'].get()=='bounded_curvature' and float(app.vars['min_turn_radius_m'].get())==.6)
    check('car_has_no_pivots',not any(e['kind']=='TURN' for e in app.result.events))
    check('car_disables_pivot_checkbox',str(app.pivot_box['state'])=='disabled')
    app.first_turn();turn=first_curve(app.result.events);app.slider.set(turn['t0']+.5*turn['duration']);pump(.2)
    ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(ROOT/'docs/assets/gui_dubins_follow.png')
    check('equivalent_steering_indicator_visible',app.front_wheel.get_visible())
    target=out/'gui_curve_export.gif';gm.filedialog.asksaveasfilename=lambda **kw:str(target)
    app.export_gif();check('GIF_button_launches_owned_renderer',app.export_process is not None)
    end=time.monotonic()+40
    while app.export_process is not None and time.monotonic()<end:pump(.05)
    check('GIF_export_finishes',app.export_process is None and target.exists() and target.stat().st_size>10000)
    notices=[];gm.messagebox.showerror=lambda title,msg:notices.append((title,msg))
    app.vars['min_turn_radius_m'].set('-1');app.start_run()
    check('negative_radius_rejected',app.process is None and bool(notices))
    app.preset_value.set(next(r['name'] for r in app.preset_rows if r['id']=='03'));app.apply_preset();app.vars['map_mode'].set('generated')
    app.vars['width'].set('28');app.vars['height'].set('22');app.vars['cell_area'].set('.12')
    app.start_run();p=app.process;folder=app.run_dir;app.cancel_run();wait_worker()
    check('cancel_does_not_publish_success',json.loads((folder/'status.json').read_text())['status']=='cancelled')
    app.close();check('owned_worker_reaped',p.poll() is not None)
except Exception:
    import traceback
    rows.append({'passed':False,'error':traceback.format_exc()})
    try:app.close()
    except Exception:pass
write_json(out/'gui_curves.json',{'platform':sys.platform,'tests':rows,'passed':sum(r['passed'] for r in rows),'total':len(rows)})
print(json.dumps(rows,indent=2))
raise SystemExit(0 if all(r['passed'] for r in rows) else 1)
