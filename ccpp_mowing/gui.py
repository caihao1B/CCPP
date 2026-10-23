"""Tk GUI: subprocess planning, main-thread UI, real-time event animation.

Closing or cancelling terminates only the child owned by this app. No ROS,
network service or external renderer is needed. A saved-status check prevents
partial writes from being presented as a finished experiment.
"""
from __future__ import annotations
from datetime import datetime
import json, os, queue, subprocess, sys, threading, time, uuid
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.patches import Polygon as MplPolygon, Circle
from .models import Config, Robot, SOLVERS, INTER_SOLVERS, PLANNERS, MOTION_MODELS, TRAJECTORY_METHODS, TURN_POLICIES
from .maps import SHAPES, DENSITIES, DISTRIBUTIONS
from .pipeline import write_json, SCHEMA
from .snapshot import load_view
from .motion import evaluate_events
from .geometry import rectangle
from .visualization import draw_scene

ROOT=Path(__file__).resolve().parent.parent

class App:
    def __init__(self,root):
        self.root=root;root.title('CCPP Mowing Planner v2.1 | Constrained curves and robot-motion replay')
        root.geometry('1400x920');root.minsize(1120,760)
        self.export_process=None;self.export_target=None;self.process=None;self.cancel_requested=False;self.result=None;self.run_dir=None;self.log_queue=queue.Queue()
        self.playing=False;self.animation_job=None;self.time_value=0.;self.last_wall=0.;self.closing=False;self.background=None
        self.vars={};self.map_path=tk.StringVar();self.robot_path=tk.StringVar();self.folder_label=tk.StringVar(value='No result loaded')
        panel=ttk.Panedwindow(root,orient='horizontal');panel.pack(fill='both',expand=True)
        left=ttk.Frame(panel);panel.add(left,weight=0)
        footer=ttk.Frame(left,padding=(10,4));footer.pack(side='bottom',fill='x')
        self.left_tabs=ttk.Notebook(left);self.left_tabs.pack(side='top',fill='both',expand=True)
        coverage_options=ttk.Frame(self.left_tabs);motion_options=ttk.Frame(self.left_tabs)
        self.left_tabs.add(coverage_options,text='Map / coverage');self.left_tabs.add(motion_options,text='Motion / curves')
        scroll_area=ttk.Frame(coverage_options);scroll_area.pack(side='top',fill='both',expand=True)
        self.options_canvas=tk.Canvas(scroll_area,width=334,highlightthickness=0)
        scrollbar=ttk.Scrollbar(scroll_area,orient='vertical',command=self.options_canvas.yview)
        scrollbar.pack(side='right',fill='y');self.options_canvas.pack(side='left',fill='both',expand=True)
        self.options_canvas.configure(yscrollcommand=scrollbar.set)
        controls=ttk.Frame(self.options_canvas,padding=10)
        option_window=self.options_canvas.create_window((0,0),window=controls,anchor='nw')
        controls.bind('<Configure>',lambda e:self.options_canvas.configure(scrollregion=self.options_canvas.bbox('all')))
        self.options_canvas.bind('<Configure>',lambda e:self.options_canvas.itemconfigure(option_window,width=e.width))
        right=ttk.Frame(panel);panel.add(right,weight=1)
        ttk.Label(controls,text='CCPP MOWING PLANNER',font=('TkDefaultFont',13,'bold')).pack(anchor='w')
        self.vehicle_label=ttk.Label(controls,text='Pure Python • offline • SI units\nDefault: 1.10 × 1.05 m | cutter: 0.70 m\nLoaded result dimensions are shown below.',wraplength=290)
        self.vehicle_label.pack(anchor='w',pady=(4,10))
        def combo(key,label,choices,default):
            row=ttk.Frame(controls);row.pack(fill='x',pady=2)
            ttk.Label(row,text=label,width=16).pack(side='left')
            v=self.vars.get(key) or tk.StringVar(value=default);self.vars[key]=v
            box=ttk.Combobox(row,textvariable=v,values=choices,width=19,state='readonly');box.pack(side='right')
            return box
        def entry(key,label,default):
            row=ttk.Frame(controls);row.pack(fill='x',pady=2)
            ttk.Label(row,text=label,width=16).pack(side='left');v=tk.StringVar(value=str(default));self.vars[key]=v
            ttk.Entry(row,textvariable=v,width=22).pack(side='right')
        combo('map_mode','Map source',('generated','original','imported'),'generated')
        self.map_button=ttk.Button(controls,text='Choose GeoJSON...',command=self.choose_map);self.map_button.pack(fill='x',pady=2)
        combo('shape','Field shape',SHAPES,'rectangle');combo('density','Obstacle density',DENSITIES,'medium')
        combo('distribution','Distribution',DISTRIBUTIONS,'uniform')
        entry('width','Width [m]',18);entry('height','Height [m]',14);entry('seed','Random seed',7)
        ttk.Separator(controls).pack(fill='x',pady=8)
        combo('grid_type','Grid',('square','triangle'),'square')
        combo('scale_mode','Grid comparison',('equal_area','equal_side','tool_cover'),'equal_area')
        entry('cell_area','Cell area [m²]',.12);entry('cell_side','Cell side [m]',.35)
        combo('partition','Partition',('bfs','kmeans','none'),'bfs');entry('cluster_size','Max cells/cluster',48)
        combo('solver','Intra-cluster',SOLVERS,'serpentine');combo('cluster_solver','Inter-cluster',INTER_SOLVERS,'nn2opt')
        combo('planner','Connector',PLANNERS,'astar');combo('timing','Time profile',('slsqp','conservative'),'slsqp')
        combo('fallback_policy','Failed query',('astar_reported','strict'),'astar_reported')
        self.allow_partial=tk.BooleanVar(value=True);self.return_home=tk.BooleanVar(value=True)
        ttk.Checkbutton(controls,text='Allow explicitly reported residual tasks',variable=self.allow_partial).pack(anchor='w',pady=2)
        ttk.Checkbutton(controls,text='Return to start',variable=self.return_home).pack(anchor='w')
        # Separate tab keeps motion constraints visible without hiding coverage controls.
        motion_area=ttk.Frame(motion_options);motion_area.pack(fill='both',expand=True)
        self.motion_canvas=tk.Canvas(motion_area,width=334,highlightthickness=0)
        motion_scroll=ttk.Scrollbar(motion_area,orient='vertical',command=self.motion_canvas.yview)
        motion_scroll.pack(side='right',fill='y');self.motion_canvas.pack(side='left',fill='both',expand=True)
        self.motion_canvas.configure(yscrollcommand=motion_scroll.set)
        controls=ttk.Frame(self.motion_canvas,padding=10)
        motion_window=self.motion_canvas.create_window((0,0),window=controls,anchor='nw')
        controls.bind('<Configure>',lambda e:self.motion_canvas.configure(scrollregion=self.motion_canvas.bbox('all')))
        self.motion_canvas.bind('<Configure>',lambda e:self.motion_canvas.itemconfigure(motion_window,width=e.width))
        ttk.Label(controls,text='ROBOT MOTION & CURVES',font=('TkDefaultFont',12,'bold')).pack(anchor='w')
        ttk.Label(controls,text='The body follows the analytical curve.\nRadius, body clearance and motion rates are validated before playback.',wraplength=308).pack(anchor='w',pady=(4,7))
        self.preset_names=[];self.preset_rows=[]
        preset_path=ROOT/'demos'/'presets.json'
        if preset_path.exists():
            self.preset_rows=json.loads(preset_path.read_text(encoding='utf-8'))
            self.preset_names=[r['name'] for r in self.preset_rows]
        self.preset_value=tk.StringVar(value=self.preset_names[0] if self.preset_names else '')
        ttk.Label(controls,text='Reproducible demonstration preset').pack(anchor='w')
        ttk.Combobox(controls,textvariable=self.preset_value,values=self.preset_names,state='readonly',width=36).pack(fill='x',pady=2)
        ttk.Button(controls,text='Apply preset (then Generate)',command=self.apply_preset).pack(fill='x',pady=(2,7))
        ttk.Button(controls,text='Choose research vehicle JSON...',command=self.choose_robot).pack(fill='x',pady=2)
        self.robot_file_label=ttk.Label(controls,text='Default research vehicle',wraplength=306)
        self.robot_file_label.pack(anchor='w',pady=(0,5))
        model_box=combo('motion_model','Robot model',MOTION_MODELS,'differential')
        combo('trajectory_method','Geometry method',TRAJECTORY_METHODS,'bezier_slsqp')
        policy_box=combo('turn_policy','Turn treatment',TURN_POLICIES,'loops_only')
        combo('timing','Time scaling',('slsqp','conservative'),'slsqp')
        entry('min_turn_radius_m','Moving R_min [m]',.10)
        entry('blend_distance_m','Corner trim [m]',1.5)
        entry('manual_setback_m','Manual setback [m]',.8)
        entry('curve_work_speed_mps','Work cap [m/s]',.50)
        entry('curve_transit_speed_mps','Transit cap [m/s]',.80)
        entry('curve_acceleration_mps2','a_long [m/s2]',.35)
        entry('curve_jerk_mps3','j_long [m/s3]',.70)
        entry('lateral_acceleration_mps2','a_lat [m/s2]',.35)
        entry('wheelbase_m','Equivalent L [m]',.65)
        entry('steering_rate_radps','Steer rate [r/s]',.50)
        entry('curve_step_m','Sample ds [m]',.05)
        combo('curve_service_rule','Task service rule',('cutter_footprint','centre_exact'),'cutter_footprint')
        entry('max_metric_nodes','Local order budget',180)
        self.pivot_fallback=tk.BooleanVar(value=False)
        self.pivot_box=ttk.Checkbutton(controls,text='Differential: allow reported pivot fallback',variable=self.pivot_fallback)
        self.pivot_box.pack(anchor='w',pady=4)
        model_box.bind('<<ComboboxSelected>>',lambda e:self.model_changed())
        policy_box.bind('<<ComboboxSelected>>',lambda e:self.model_changed())
        self.model_changed()
        ttk.Label(controls,text='loops_only: real forward loops; no chassis pivot or reverse. Failed turns are rejected.\nloop_then_pivot: try loops first, allow explicitly reported differential pivots.\nlegacy_pivot: old baseline. Curvature jumps stop to steer in the car-like model.',wraplength=310).pack(anchor='w',pady=5)
        self.left_tabs.select(motion_options)
        btns=ttk.Frame(footer);btns.pack(fill='x',pady=(10,4))
        self.run_button=ttk.Button(btns,text='Generate path + trajectory',command=self.start_run);self.run_button.pack(side='left',fill='x',expand=True)
        self.cancel_button=ttk.Button(btns,text='Cancel',command=self.cancel_run,state='disabled');self.cancel_button.pack(side='right')
        ttk.Button(footer,text='Open saved result...',command=self.choose_result).pack(fill='x',pady=2)
        ttk.Button(footer,text='Save current path figure...',command=self.save_figure).pack(fill='x',pady=2)
        self.export_button=ttk.Button(footer,text='Export first curved turn GIF...',command=self.export_gif);self.export_button.pack(fill='x',pady=2)
        self.progress=ttk.Progressbar(footer,maximum=100);self.progress.pack(fill='x',pady=6)
        self.status=tk.StringVar(value='Choose parameters, then Generate path.');ttk.Label(footer,textvariable=self.status,wraplength=295).pack(anchor='w')
        ttk.Label(footer,textvariable=self.folder_label,wraplength=295).pack(anchor='w',pady=6)
        self.notebook=ttk.Notebook(right);self.notebook.pack(fill='both',expand=True)
        path_tab=ttk.Frame(self.notebook);self.notebook.add(path_tab,text='Path & animation')
        self.figure=Figure(figsize=(10,7),dpi=100);self.ax=self.figure.add_subplot(111)
        self.figure.subplots_adjust(bottom=.18,top=.84,left=.08,right=.98)
        path_tab.columnconfigure(0,weight=1);path_tab.rowconfigure(0,weight=1)
        self.canvas=FigureCanvasTkAgg(self.figure,path_tab);self.canvas.get_tk_widget().grid(row=0,column=0,sticky='nsew')
        toolbar=NavigationToolbar2Tk(self.canvas,path_tab,pack_toolbar=False);toolbar.grid(row=1,column=0,sticky='ew')
        self.anim_bar=ttk.Frame(path_tab,padding=4);self.anim_bar.grid(row=2,column=0,sticky='ew')
        self.play_button=ttk.Button(self.anim_bar,text='Play / pause',command=self.toggle_play);self.play_button.pack(side='left')
        ttk.Button(self.anim_bar,text='First curve / turn',command=self.first_turn).pack(side='left')
        ttk.Button(self.anim_bar,text='Heading loop',command=self.first_heading_loop).pack(side='left')
        ttk.Button(self.anim_bar,text='Reset',command=self.reset_animation).pack(side='left')
        self.speed=tk.StringVar(value='5');ttk.Combobox(self.anim_bar,textvariable=self.speed,values=('1','5','20','50','200','1000'),width=7,state='readonly').pack(side='left')
        ttk.Label(self.anim_bar,text='×').pack(side='left');self.follow=tk.BooleanVar(value=False)
        ttk.Checkbutton(self.anim_bar,text='Follow / zoom',variable=self.follow,command=self.redraw_background).pack(side='left')
        self.slider=ttk.Scale(self.anim_bar,from_=0,to=1,command=self.seek);self.slider.pack(side='left',fill='x',expand=True,padx=5)
        self.telemetry=tk.StringVar(value='No trajectory');ttk.Label(path_tab,textvariable=self.telemetry,wraplength=950).grid(row=3,column=0,sticky='ew',padx=8,pady=5)
        motion_tab=ttk.Frame(self.notebook);self.notebook.add(motion_tab,text='Speed')
        self.speed_fig=Figure(figsize=(10,5));self.speed_ax=self.speed_fig.add_subplot(111)
        self.speed_canvas=FigureCanvasTkAgg(self.speed_fig,motion_tab);self.speed_canvas.get_tk_widget().pack(fill='both',expand=True)
        report_tab=ttk.Frame(self.notebook);self.notebook.add(report_tab,text='Metrics & audit log')
        self.report=tk.Text(report_tab,wrap='word',font=('TkFixedFont',10));self.report.pack(fill='both',expand=True)
        self.ax.text(.5,.5,'Generate a plan or open a saved result.\nAll distances are metres; all movements are simulated.',ha='center',transform=self.ax.transAxes)
        self.canvas.draw()
        curve_tab=ttk.Frame(self.notebook);self.notebook.add(curve_tab,text='Curvature & constraints')
        self.curve_fig=Figure(figsize=(10,5));self.curve_ax=self.curve_fig.add_subplot(111)
        self.curve_canvas=FigureCanvasTkAgg(self.curve_fig,curve_tab);self.curve_canvas.get_tk_widget().pack(fill='both',expand=True)
        self.curve_summary=tk.StringVar(value='No curve result loaded')
        ttk.Label(curve_tab,textvariable=self.curve_summary,wraplength=950,padding=8).pack(fill='x')
        self.poll_job=root.after(100,self.poll)
        self.canvas.mpl_connect('draw_event',self.on_draw)
        root.protocol('WM_DELETE_WINDOW',self.close)
        # A reproducible, certified no-pivot example is safer than arbitrary default geometry.
        recommended=next((p for p in self.preset_rows if p['id']=='08'),None)
        if recommended:
            self.preset_value.set(recommended['name']);self.apply_preset()

    def choose_map(self):
        p=filedialog.askopenfilename(filetypes=[('GeoJSON','*.geojson'),('JSON','*.json')])
        if p:self.map_path.set(p);self.vars['map_mode'].set('imported');self.status.set(p)

    def choose_robot(self):
        p=filedialog.askopenfilename(filetypes=[('Research vehicle JSON','*.json')])
        if p:
            try:
                data=json.loads(Path(p).read_text(encoding='utf-8'))
                rb=Robot(**data.get('simulation_parameters',data)).validate()
                self.robot_path.set(p);self.robot_file_label.configure(text=f'{Path(p).name}: {rb.length_m:.2f} x {rb.width_m:.2f} m')
            except Exception as e:messagebox.showerror('Vehicle refused',str(e))

    def request(self):
        v=lambda k:self.vars[k].get()
        cfg=Config(grid_type=v('grid_type'),scale_mode=v('scale_mode'),cell_area=float(v('cell_area')),cell_side=float(v('cell_side')),
            solver=v('solver'),partition=v('partition'),cluster_size=int(v('cluster_size')),cluster_solver=v('cluster_solver'),
            planner=v('planner'),seed=int(v('seed')),timing=v('timing'),fallback_policy=v('fallback_policy'),
            allow_partial=self.allow_partial.get(),return_to_start=self.return_home.get(),
            motion_model=v('motion_model'),trajectory_method=v('trajectory_method'),turn_policy=v('turn_policy'),
            min_turn_radius_m=float(v('min_turn_radius_m')),blend_distance_m=float(v('blend_distance_m')),
            manual_setback_m=float(v('manual_setback_m')),curve_work_speed_mps=float(v('curve_work_speed_mps')),
            curve_transit_speed_mps=float(v('curve_transit_speed_mps')),curve_acceleration_mps2=float(v('curve_acceleration_mps2')),
            curve_jerk_mps3=float(v('curve_jerk_mps3')),lateral_acceleration_mps2=float(v('lateral_acceleration_mps2')),
            wheelbase_m=float(v('wheelbase_m')),steering_rate_radps=float(v('steering_rate_radps')),
            curve_step_m=float(v('curve_step_m')),curve_service_rule=v('curve_service_rule'),
            max_metric_nodes=int(v('max_metric_nodes')),curve_allow_pivot_fallback=self.pivot_fallback.get()).validate()
        args=['--shape',v('shape'),'--density',v('density'),'--distribution',v('distribution'),
              '--width',str(float(v('width'))),'--height',str(float(v('height')))]
        if v('map_mode')=='original':args+=['--map',str(ROOT/'maps'/'original_field.geojson')]
        if v('map_mode')=='imported':
            if not self.map_path.get():raise ValueError('Choose an imported map file first')
            args+=['--map',self.map_path.get()]
        if self.robot_path.get():
            data=json.loads(Path(self.robot_path.get()).read_text(encoding='utf-8'))
            rb=Robot(**data.get('simulation_parameters',data)).validate()
            cfg.tool_radius=rb.cutter_radius
            args+=['--robot',self.robot_path.get()]
        return cfg,args

    def start_run(self):
        if self.process is not None:return
        try:cfg,args=self.request()
        except Exception as e:messagebox.showerror('Invalid configuration',str(e));return
        self.pause();self.cancel_requested=False;stamp=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:5]
        self.run_dir=ROOT/'outputs'/('gui_'+stamp);self.run_dir.mkdir(parents=True)
        request=self.run_dir/'request.json';write_json(request,cfg.to_dict())
        command=[sys.executable,'-m','ccpp_mowing','plan','--config',str(request),'--out',str(self.run_dir),*args]
        env=os.environ.copy();env['MPLBACKEND']='Agg';env['PYTHONUNBUFFERED']='1'
        env['PYTHONUTF8']='1';env['PYTHONIOENCODING']='utf-8';env['OPENBLAS_NUM_THREADS']='1';env['PYTHONPATH']=str(ROOT)+os.pathsep+env.get('PYTHONPATH','')
        try:self.process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
        except Exception as e:messagebox.showerror('Launch failed',str(e));self.process=None;return
        proc=self.process
        def read_output():
            for line in proc.stdout:self.log_queue.put(line.rstrip())
        threading.Thread(target=read_output,daemon=True).start()
        self.run_button.configure(state='disabled');self.cancel_button.configure(state='normal')
        self.progress['value']=0;self.status.set('Planning in an isolated worker process...');self.folder_label.set(str(self.run_dir))

    def cancel_run(self):
        if self.process is None:return
        self.cancel_requested=True;(self.run_dir/'cancel.flag').touch();self.status.set('Cancelling; incomplete result will not be loaded')
        proc=self.process;folder=self.run_dir
        def force():
            if proc.poll() is None:proc.terminate()
            write_json(folder/'status.json',{'schema':SCHEMA,'status':'cancelled','validation_passed':False})
        self.root.after(1200,force)

    def poll(self):
        if self.closing:return
        while True:
            try:line=self.log_queue.get_nowait()
            except queue.Empty:break
            try:
                item=json.loads(line)
                if 'progress' in item:
                    p=item['progress'];self.status.set(p['message']);self.progress['value']=100*p['fraction']
                elif 'error' in item:self.status.set(item['error'])
            except (ValueError,KeyError):pass
            self.report.insert('end',line+'\n');self.report.see('end')
        if self.process is not None and self.process.poll() is not None:
            code=self.process.returncode;self.process=None
            self.run_button.configure(state='normal');self.cancel_button.configure(state='disabled')
            if self.cancel_requested:
                write_json(self.run_dir/'status.json',{'schema':SCHEMA,'status':'cancelled','validation_passed':False})
                self.status.set('Cancelled. No completion claimed.')
            elif code==0:
                try:self.load(self.run_dir)
                except Exception as e:self.status.set(f'Output refused: {e}')
            else:self.status.set(f'Worker stopped (exit {code}); read status.json / error.log. No completion claimed.')
        if self.export_process is not None and self.export_process.poll() is not None:
            proc=self.export_process;self.export_process=None;self.export_button.configure(state='normal')
            text=proc.stdout.read() if proc.stdout else ''
            self.report.insert('end','\nGIF export: '+text+'\n')
            self.status.set(('GIF saved: '+str(self.export_target)) if proc.returncode==0 else 'GIF export failed; see audit log')
        self.poll_job=self.root.after(100,self.poll)

    def choose_result(self):
        p=filedialog.askdirectory(initialdir=ROOT/'examples')
        if p:
            try:self.load(p)
            except Exception as e:messagebox.showerror('Result refused',str(e))

    def load(self,folder):
        self.pause();self.result=load_view(folder);self.sync_config(self.result.config)
        self.vars['map_mode'].set('imported');self.map_path.set(str(Path(folder).resolve()/'map.geojson'))
        self.folder_label.set(str(Path(folder).resolve()))
        draw_scene(self.ax,self.result)
        self.body=MplPolygon(np.zeros((4,2)),closed=True,fill=False,linewidth=2.,animated=True,zorder=20)
        self.ax.add_patch(self.body);self.cutter=Circle((0,0),self.result.robot.cutter_radius,fill=False,linestyle=':',animated=True,zorder=20)
        self.ax.add_patch(self.cutter);self.heading,=self.ax.plot([],[],linewidth=2.,animated=True,zorder=21)
        self.front_wheel,=self.ax.plot([],[],linewidth=3.,animated=True,zorder=23)
        self.trail,=self.ax.plot([],[],linewidth=1.8,alpha=.8,animated=True,zorder=19)
        self.full_limits=(self.ax.get_xlim(),self.ax.get_ylim());self.follow.set(False)
        duration=self.result.events[-1]['t1'];self.slider.configure(to=duration)
        values=evaluate_events(self.result.events,np.linspace(0,duration,min(12000,max(1000,int(duration/2)))),self.result.robot)
        self.speed_ax.clear();self.speed_ax.plot(values[:,0]/60,values[:,4],linewidth=.65,label='Planned forward speed')
        self.speed_ax.axhline(self.result.robot.transit_speed_mps,linestyle='--',label='Transit limit')
        self.speed_ax.set(xlabel='Simulated time [min]',ylabel='Speed [m/s]',title='Not measured or closed-loop vehicle motion')
        self.speed_ax.legend();self.speed_fig.tight_layout();self.speed_canvas.draw()
        self.report.delete('1.0','end');self.report.insert('end',json.dumps({'metrics':self.result.metrics,'validation':self.result.validation,'ordering':self.result.order.stats,'trajectory_optimization':self.result.optimization},ensure_ascii=False,indent=2))
        self.curve_ax.clear()
        if self.result.config.trajectory_method!='stop_turn_go':
            from .curve_motion import curve_diagnostics
            chunks=[np.linspace(e['t0'],e['t1'],25,endpoint=False) for e in self.result.events if 'curve' in e]
            times=np.concatenate(chunks) if chunks else np.array([])
            if len(times):
                diag=curve_diagnostics(self.result.events,times,self.result.robot,self.result.config)
                self.curve_ax.plot(times/60,diag[:,1],linewidth=.6,label='Moving-path curvature')
            bound=1/self.result.config.min_turn_radius_m
            self.curve_ax.axhline(bound,linestyle='--',label='+/- 1/R_min');self.curve_ax.axhline(-bound,linestyle='--')
            self.curve_ax.legend()
            r=self.result.validation
            self.curve_summary.set(f"Continuous bounds checked: {r.get('valid')} | certified maximum curvature: {r.get('maximum_certified_curvature_inv_m',0):.4f} 1/m | "
                f"pivots: {self.result.metrics.get('pivot_fallbacks',0)}. Curvature applies to translation, not a differential pivot.")
        else:self.curve_summary.set('Legacy stop-turn-go: straight translation and finite-rate in-place rotation; no moving curve.')
        self.curve_ax.set(xlabel='Simulated time [min]',ylabel='Curvature [1/m]',title='Plot samples visualize the curve; independent certificates enforce the constraints')
        self.curve_fig.tight_layout();self.curve_canvas.draw()
        self.canvas.draw();self.time_value=0.;self.slider.set(0);self.draw_robot()
        m=self.result.metrics;self.status.set(f"Validated {m['serviced_centres']}/{m['candidate_centres']} targets ({m.get('service_rule','centre_exact')}); manual area {m['manual_handover_area_m2']:.2f} m²")
        self.progress['value']=100

    def on_draw(self,event=None):
        if self.result is not None:self.background=self.canvas.copy_from_bbox(self.ax.bbox)

    def redraw_background(self):
        if self.result is None:return
        if not self.follow.get():self.ax.set_xlim(*self.full_limits[0]);self.ax.set_ylim(*self.full_limits[1])
        self.canvas.draw();self.draw_robot()

    def draw_robot(self):
        if self.result is None:return
        rb=self.result.robot
        self.vehicle_label.configure(text=f'Pure Python • offline • SI units\nResult: {rb.length_m:.2f} × {rb.width_m:.2f} m | cutter: {rb.cutting_width_m:.2f} m\nReference replay; no hardware control')
        a=evaluate_events(self.result.events,[self.time_value],self.result.robot)[0];p=a[1:3];yaw=a[3]
        poly=rectangle(p,yaw,self.result.robot.length_m,self.result.robot.width_m)
        self.body.set_xy(np.asarray(poly.exterior.coords));self.cutter.center=p;self.cutter.set_visible(bool(a[12]))
        self.heading.set_data([p[0],p[0]+.75*np.cos(yaw)],[p[1],p[1]+.75*np.sin(yaw)])
        from .curve_motion import curve_diagnostics
        diag=curve_diagnostics(self.result.events,[self.time_value],self.result.robot,self.result.config)[0]
        tip=p+.36*np.array([np.cos(yaw),np.sin(yaw)]);steer=yaw+diag[2]
        self.front_wheel.set_data([tip[0]-.16*np.cos(steer),tip[0]+.16*np.cos(steer)],
                                 [tip[1]-.16*np.sin(steer),tip[1]+.16*np.sin(steer)])
        self.front_wheel.set_visible(self.result.config.motion_model=='bounded_curvature')
        tail=evaluate_events(self.result.events,np.linspace(max(0,self.time_value-12),self.time_value,45),self.result.robot)
        self.trail.set_data(tail[:,1],tail[:,2])
        if self.follow.get():
            left,right=self.ax.get_xlim();bottom,top=self.ax.get_ylim()
            if not (left+1<p[0]<right-1 and bottom+1<p[1]<top-1) or right-left>6.01:
                self.ax.set_xlim(p[0]-3,p[0]+3);self.ax.set_ylim(p[1]-2.4,p[1]+2.4);self.canvas.draw()
        if self.background is not None:
            self.canvas.restore_region(self.background)
            for artist in (self.body,self.cutter,self.heading,self.front_wheel,self.trail):self.ax.draw_artist(artist)
            self.canvas.blit(self.ax.bbox)
        e=self.result.events[int(a[13])]
        self.telemetry.set(f"t={self.time_value/60:.2f} min | {e['kind']} | v={a[4]:.3f} m/s | yaw rate={a[5]:.3f} rad/s | "
                           f"kappa={diag[1]:.3f} 1/m | a_lat={diag[4]:.3f} m/s2 | L/R={a[8]:.3f}/{a[9]:.3f} | {e['reason']}")

    def seek(self,value):
        if self.result is None:return
        self.time_value=float(value);self.draw_robot()

    def toggle_play(self):
        if self.result is None:return
        if self.playing:self.pause();return
        self.playing=True;self.last_wall=time.perf_counter();self.animate()

    def animate(self):
        if not self.playing or self.result is None or self.closing:return
        now=time.perf_counter();dt=min(.25,now-self.last_wall);self.last_wall=now
        self.time_value=min(self.result.events[-1]['t1'],self.time_value+dt*float(self.speed.get()))
        self.slider.set(self.time_value)
        if self.time_value>=self.result.events[-1]['t1']:self.pause();return
        self.animation_job=self.root.after(40,self.animate)

    def pause(self):
        self.playing=False
        if self.animation_job is not None:
            try:self.root.after_cancel(self.animation_job)
            except tk.TclError:pass
            self.animation_job=None

    def reset_animation(self):self.pause();self.slider.set(0)

    def first_turn(self):
        self.pause()
        if self.result:
            from .animation_export import first_curve
            turn=first_curve(self.result.events)
            if turn:self.slider.set(max(0,turn['t0']-2));self.speed.set('1');self.follow.set(True);self.redraw_background()

    def first_heading_loop(self):
        self.pause()
        if not self.result:return
        e=next((e for e in self.result.events if e.get('reason')=='certified_dubins_heading_loop'),None)
        if e:
            self.slider.set(max(0,e['t0']-.5));self.speed.set('1');self.follow.set(True);self.redraw_background()
        else:self.status.set('This result has no same-point heading loop. Use First curve / turn for cross-row turns.')

    def save_figure(self):
        if self.result is None:return
        p=filedialog.asksaveasfilename(defaultextension='.png',filetypes=[('PNG image','*.png')])
        if p:
            artists=(self.body,self.cutter,self.heading,self.front_wheel,self.trail)
            for a in artists:a.set_animated(False)
            self.figure.savefig(p,dpi=160)
            for a in artists:a.set_animated(True)
            self.canvas.draw();self.draw_robot()

    def sync_config(self,cfg):
        for key,value in cfg.to_dict().items():
            if key in self.vars:self.vars[key].set(str(value))
        self.allow_partial.set(cfg.allow_partial);self.return_home.set(cfg.return_to_start)
        self.pivot_fallback.set(cfg.curve_allow_pivot_fallback)
        self.model_changed()

    def model_changed(self):
        bounded=(self.vars['motion_model'].get()=='bounded_curvature' or self.vars['turn_policy'].get()=='loops_only')
        self.pivot_box.configure(state='disabled' if bounded else 'normal')
        if bounded:
            self.pivot_fallback.set(False)
            if self.vars['trajectory_method'].get()=='stop_turn_go':self.vars['trajectory_method'].set('dubins')

    def apply_preset(self):
        try:
            item=next(r for r in self.preset_rows if r['name']==self.preset_value.get())
            cfg=Config.from_dict(json.loads((ROOT/item['config']).read_text(encoding='utf-8')))
            self.sync_config(cfg);self.vars['map_mode'].set('imported');self.map_path.set(str(ROOT/item['map']))
            self.robot_path.set(str(ROOT/item['robot']) if item.get('robot') else '')
            self.robot_file_label.configure(text=Path(item['robot']).name if item.get('robot') else 'Default research vehicle')
            self.status.set('Preset applied. Generate to recompute; the displayed old result has not changed.')
        except Exception as e:messagebox.showerror('Preset refused',str(e))

    def export_gif(self):
        if self.result is None or self.export_process is not None:return
        p=filedialog.asksaveasfilename(defaultextension='.gif',filetypes=[('Animated GIF','*.gif')])
        if not p:return
        self.export_target=Path(p)
        env=os.environ.copy();env.update(MPLBACKEND='Agg',PYTHONUTF8='1',PYTHONIOENCODING='utf-8',OPENBLAS_NUM_THREADS='1')
        self.export_process=subprocess.Popen([sys.executable,'-m','ccpp_mowing','render',str(self.result.folder),'--out',str(p)],
            cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
        self.export_button.configure(state='disabled');self.status.set('Rendering the analytical curve to GIF...')

    def close(self):
        self.closing=True;self.pause()
        try:self.root.after_cancel(self.poll_job)
        except tk.TclError:pass
        if self.process is not None and self.process.poll() is None:
            (self.run_dir/'cancel.flag').touch();self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3)
            write_json(self.run_dir/'status.json',{'schema':SCHEMA,'status':'cancelled','validation_passed':False})
        if self.export_process is not None and self.export_process.poll() is None:
            self.export_process.terminate()
            try:self.export_process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.export_process.kill();self.export_process.wait(timeout=3)
        self.root.destroy()


def launch():
    root=tk.Tk();App(root);root.mainloop()
