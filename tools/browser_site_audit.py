from pathlib import Path
import json,sys,threading,time,math,urllib.request,argparse,shutil
from functools import partial
from http.server import ThreadingHTTPServer,SimpleHTTPRequestHandler
from playwright.sync_api import sync_playwright
import numpy as np
parser=argparse.ArgumentParser(description='Optional browser QA; requires Playwright. Not needed to view the website.')
parser.add_argument('--site',type=Path,required=True)
parser.add_argument('--html-injection',action='store_true',help='Render real HTML with HTTP assets when managed browser policy blocks navigation')
parser.add_argument('--chromium',default=None)
args=parser.parse_args()
project=Path(__file__).resolve().parents[1];ROOT=project.parent;site=args.site.resolve();out=project/'docs/qa_v2_1';out.mkdir(exist_ok=True);(ROOT/'previews').mkdir(exist_ok=True)
sys.path.insert(0,str(project))
from ccpp_mowing.models import Robot
from ccpp_mowing.motion import evaluate_events
class Quiet(SimpleHTTPRequestHandler):
 def log_message(self,*a):pass
server=ThreadingHTTPServer(('127.0.0.1',0),partial(Quiet,directory=str(site)))
threading.Thread(target=server.serve_forever,daemon=True).start();base=f'http://127.0.0.1:{server.server_port}/'
checks=[];errors=[];numerics={}
def ck(name,ok,**data):
 checks.append(dict(name=name,passed=bool(ok),**data));assert ok,(name,data)
with sync_playwright() as p:
 b=p.chromium.launch(executable_path=args.chromium or shutil.which('chromium'),headless=True,args=['--no-sandbox'])
 def load(name,width=1440,height=1000):
  pg=b.new_page(viewport={'width':width,'height':height},device_scale_factor=1);pg.set_default_timeout(7000)
  pg.on('pageerror',lambda e:errors.append(str(e)))
  # Managed Chromium blocks all top-level navigation. Render the actual response HTML
  # with its normal HTTP asset base; no policy changes or fabricated assets.
  with urllib.request.urlopen(base+name) as r:html=r.read().decode('utf-8')
  if args.html_injection:pg.set_content(html.replace('<head>','<head><base href="'+base+'">'),wait_until='load')
  else:pg.goto(base+name,wait_until='load')
  if name=='ccpp.html':pg.wait_for_function('window.CCPPReplay && window.CCPP_CURRENT_POSE')
  pg.wait_for_timeout(120);return pg
 pg=load('ccpp.html');ck('ten_real_cases',pg.evaluate('CCPP_CASES.length')==10);ck('six_grid_algorithm_rows',pg.locator('#comparisonRows tr').count()==6)
 ids=pg.evaluate('CCPP_CASES.map(d=>d.id)')
 for cid in ids:
  pg.select_option('#caseSelect',cid)
  d=json.loads((project/'comparison_examples'/cid/'comparison_case.json').read_text());ev=json.loads((project/'comparison_examples'/cid/'events.json').read_text());rb=Robot(**d['robot'])
  ck(cid+':full_order_retained',pg.locator('#visitTable tr').count()==d['planned_visits'])
  ck(cid+':no_hidden_pivot',pg.evaluate('CCPPReplay.state.data.events.every(e=>e.kind!=="TURN")'))
  for mode in ['grid','order','curve']:
   pg.locator(f'[data-stage={mode}]').click();ck(cid+':stage_'+mode,pg.evaluate('CCPPReplay.state.stage')==mode)
  pg.locator('#loopButton').click();win=pg.evaluate('CCPPReplay.state.focus');ck(cid+':full_heading_loop_window',bool(win and abs(win['delta'])>3 and (cid.startswith('car_') or win['id'].startswith('heading_'))))
  times=np.linspace(0,ev[-1]['t1'],101).tolist()
  for e in ev[::max(1,len(ev)//30)]:times.extend([max(0,e['t0']-1e-6),e['t0'],min(ev[-1]['t1'],e['t0']+1e-6)])
  exp=evaluate_events(ev,times,rb)[:,1:6]
  got=np.array(pg.evaluate('ts=>ts.map(t=>{let a=CCPPReplay.poseAt(CCPPReplay.state.data,t);return[a.x,a.y,a.yaw,a.v,a.w]})',times))
  diff=got-exp;diff[:,2]=np.arctan2(np.sin(diff[:,2]),np.cos(diff[:,2]));err=float(np.max(abs(diff)))
  numerics[cid]=dict(samples=len(times),max_abs_error=err);ck(cid+':JS_vs_Python',err<1e-8,samples=len(times),max_abs_error=err)
  pg.locator('#curveButton').click();ck(cid+':curve_zoom_available',pg.evaluate('CCPPReplay.state.focus.t1>CCPPReplay.state.focus.t0'))
  pg.locator('#fullButton').click();ck(cid+':full_view_resets',pg.evaluate('CCPPReplay.state.focus===null && CCPPReplay.state.t===0'))
 # Same-order checks use actual saved order hashes.
 hashes={cid:json.loads((project/'comparison_examples'/cid/'comparison_case.json').read_text())['order_sha256'] for cid in ids}
 ck('fixed_order_ablation_hashes_equal',len({hashes[x] for x in ['square_ga','square_ga_circular','square_ga_conservative']})==1)
 ck('car_radius_order_hashes_equal',hashes['car_r060']==hashes['car_r090'])
 pg.select_option('#caseSelect','square_mst');pg.locator('#loopButton').click();pg.locator('#playButton').click();t0=pg.evaluate('CCPPReplay.state.t');pg.wait_for_timeout(350);ck('replay_time_advances',pg.evaluate('CCPPReplay.state.t')>t0);pg.locator('#playButton').click();ts=pg.evaluate('CCPPReplay.state.t');pg.wait_for_timeout(100);ck('pause_freezes_time',pg.evaluate('CCPPReplay.state.t')==ts)
 pg.evaluate('CCPPReplay.state.t=CCPPReplay.state.focus.t1-.01');pg.locator('#playButton').click();pg.wait_for_timeout(250);ck('end_stops_without_pose_reset',pg.evaluate('!CCPPReplay.state.playing && CCPPReplay.state.t===CCPPReplay.state.focus.t1'))
 pg.locator('#resetButton').click();ck('explicit_reset_goes_to_focus_start',pg.evaluate('CCPPReplay.state.t===CCPPReplay.state.focus.t0'))
 pg.locator('#fullButton').click();pg.locator('#visitTable tr').nth(9).click();ck('node_table_selects_real_sequence',pg.locator('#visitStart').input_value()=='10' and pg.evaluate('CCPPReplay.state.stage')=='order')
 for i,vid in enumerate(pg.locator('video').all()):
  vid.scroll_into_view_if_needed();pg.wait_for_function('(i)=>document.querySelectorAll("video")[i].readyState>=2',arg=i)
  ck(f'video_{i}:metadata_decoded',vid.evaluate('(v)=>v.videoWidth>0&&v.duration>20'))
  vid.evaluate('(v)=>{v.muted=true;return v.play()}');pg.wait_for_timeout(250);ck(f'video_{i}:actually_plays',vid.evaluate('(v)=>v.currentTime>0'));vid.evaluate('(v)=>v.pause()')
 ck('no_autolooping_videos',pg.locator('video[loop]').count()==0)
 pg.select_option('#caseSelect','square_ga');pg.locator('#demo').scroll_into_view_if_needed();pg.wait_for_timeout(200);pg.screenshot(path=str(ROOT/'previews/lab_final_desktop.png'),full_page=False)
 pg.select_option('#caseSelect','square_mst');pg.locator('#loopButton').click();pg.evaluate('CCPPReplay.state.t=CCPPReplay.state.focus.t0+(CCPPReplay.state.focus.t1-CCPPReplay.state.focus.t0)*.45;CCPPReplay.draw()');pg.screenshot(path=str(ROOT/'previews/loop_final_desktop.png'))
 pg.close()
 for width,height in [(1440,1000),(768,1024),(390,844),(360,800)]:
  for name in ['index.html','ccpp.html','projects.html']:
   pg=load(name,width,height);sw=pg.evaluate('document.documentElement.scrollWidth');ck(f'layout:{name}:{width}',sw<=width+1,scroll_width=sw)
   if width==1440 and name=='index.html':pg.screenshot(path=str(ROOT/'previews/home_final_desktop.png'),full_page=True)
   if width==390:
    if name=='index.html':
     pg.locator('.menu-btn').click();ck('mobile_menu_opens',pg.locator('.menu-btn').get_attribute('aria-expanded')=='true');pg.locator('.menu-btn').click();pg.screenshot(path=str(ROOT/'previews/home_final_mobile.png'),full_page=True)
    if name=='ccpp.html':pg.locator('#demo').scroll_into_view_if_needed();pg.screenshot(path=str(ROOT/'previews/lab_final_mobile.png'))
   pg.close()
 ck('no_javascript_errors',not errors,errors=errors)
 b.close()
server.shutdown()
report=dict(checks=checks,passed=sum(x['passed'] for x in checks),total=len(checks),numerical=numerics,notes='Actual HTML with unchanged loopback HTTP assets; html_injection='+str(args.html_injection)+'; platform='+sys.platform+'; not a live GitHub Pages deployment test.')
(out/'browser.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps({k:report[k] for k in ('passed','total','numerical','notes')},indent=2))
