"""Exercise v2.1 widgets and a real child planner; run under Xvfb on headless Linux."""
from pathlib import Path
import sys,json,time,os
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import tkinter as tk
from PIL import ImageGrab
from ccpp_mowing.gui import App
import ccpp_mowing.gui as gm
from ccpp_mowing.motion import evaluate_events
root=tk.Tk();app=App(root);checks=[];errors=[]
gm.messagebox.showerror=lambda title,msg:errors.append((title,msg))
def pump(t=.1):
 end=time.monotonic()+t
 while time.monotonic()<end:root.update();time.sleep(.005)
def ck(name,value):
 assert value,name
 checks.append(dict(name=name,passed=True))
try:
 pump(.3)
 ck('default_reproducible_preset_08',app.preset_value.get().startswith('08'))
 ck('loops_only_is_selected',app.vars['turn_policy'].get()=='loops_only')
 ck('pivot_disabled',str(app.pivot_box['state'])=='disabled' and not app.pivot_fallback.get())
 ck('custom_vehicle_selected',Path(app.robot_path.get()).name=='robot_08.json')
 app.run_button.invoke();ck('GUI_launches_actual_planner',app.process is not None)
 end=time.monotonic()+90
 while app.process is not None and time.monotonic()<end:pump(.05)
 ck('worker_finished',app.process is None and not errors)
 ck('result_valid',app.result is not None and app.result.validation['valid'])
 d=app.result
 ck('selected_vehicle_used',d.robot.length_m==.65 and d.robot.width_m==.48)
 ck('no_chassis_pivot',all(e['kind']!='TURN' for e in d.events))
 ck('real_heading_loops',d.metrics['heading_loops']>0 if 'heading_loops' in d.metrics else any((e.get('turn_id') or '').startswith('heading_') for e in d.events))
 app.first_heading_loop();pump()
 ck('heading_loop_button_slow_follow',app.follow.get() and app.speed.get()=='1')
 es=[e for e in d.events if (e.get('turn_id') or '').startswith('heading_')];key=es[0]['turn_id'];group=[e for e in es if e['turn_id']==key]
 ts=np.linspace(group[0]['t0'],group[-1]['t1'],25);poses=evaluate_events(d.events,ts,d.robot)
 ck('loop_leaves_turning_point',np.max(np.linalg.norm(poses[:,1:3]-group[0]['p0'],axis=1))>.1)
 ck('forward_speed_nonnegative',min(poses[:,4])>=-1e-9)
 for t,a in zip(ts[::4],poses[::4]):
  app.slider.set(float(t));pump(.02)
  assert np.linalg.norm(app.body.get_xy()[:4].mean(axis=0)-a[1:3])<1e-8
  x,y=app.heading.get_data();yaw=np.arctan2(y[-1]-y[0],x[-1]-x[0]);assert abs(np.arctan2(np.sin(yaw-a[3]),np.cos(yaw-a[3])))<1e-8
 ck('body_and_heading_match_analytic_loop',True)
 app.slider.set(float(ts[12]));pump(.1)
 out=ROOT/'docs/qa_v2_1';out.mkdir(exist_ok=True)
 ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(out/'gui_heading_loop.png')
 app.preset_value.set(next(r['name'] for r in app.preset_rows if r['id']=='09'));app.apply_preset()
 ck('triangle_preset_keeps_vehicle_and_no_pivot',app.vars['grid_type'].get()=='triangle' and app.vars['turn_policy'].get()=='loops_only')
 app.load(ROOT/'comparison_examples/car_r090');pump()
 ck('saved_car_radius_restored',abs(float(app.vars['min_turn_radius_m'].get())-.9)<1e-9)
 ck('car_model_disallows_pivot',str(app.pivot_box['state'])=='disabled' and not any(e['kind']=='TURN' for e in app.result.events))
 app.close()
except Exception:
 import traceback
 checks.append(dict(name='unexpected_failure',passed=False,error=traceback.format_exc()))
 try:app.close()
 except Exception:pass
out=ROOT/'docs/qa_v2_1';out.mkdir(exist_ok=True)
r=dict(platform=sys.platform,checks=checks,passed=sum(x['passed'] for x in checks),total=len(checks),messages=errors)
(out/'gui_loops.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2));sys.exit(not all(x['passed'] for x in checks))
