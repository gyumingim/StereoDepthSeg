"""Dependency-free offline orbit viewer for exported point clouds."""
import json
import numpy as np


def write_viewer(path, points, colors, metadata):
    step = max(1, (len(points)+99999)//100000)
    data = dict(points=np.round(points[::step], 4).tolist(),
                colors=np.clip(colors[::step], 0, 255).astype(int).tolist(),
                objects=metadata['objects'], trajectory=metadata['trajectory'])
    payload = json.dumps(data, separators=(',', ':')).replace('<', '\\u003c')
    path.write_text('''<!doctype html><meta charset="utf-8"><title>Scene 3D cloud</title>
<style>body{margin:0;background:#11151b;color:#eee;font:14px system-ui}canvas{display:block;width:100vw;height:100vh;touch-action:none}aside{position:absolute;top:16px;left:16px;background:#17212ddd;padding:16px;border-radius:10px;max-height:75vh;overflow:auto}h2{margin:0 0 8px}p{color:#bdc8d8}button{padding:6px;margin-right:6px}label{display:block;margin:7px 0}</style>
<canvas id="view"></canvas><aside><h2>Scene 3D cloud · meters</h2><p>Drag: rotate · Wheel: zoom · Shift-drag: pan<br>Observed surfaces · Origin: left camera<br>X right / Y down / Z forward</p><button id="reset">Reset view</button><label><input type="checkbox" id="trail" checked>Camera trajectory</label><div id="objects"></div></aside>
<script>const D='''+payload+''';
const canvas=document.querySelector('#view'),ctx=canvas.getContext('2d');
let yaw=-.35,pitch=-.2,zoom=220,pan=[0,0],drag=null;
let center=D.points.length?D.points.reduce((a,p)=>a.map((v,i)=>v+p[i]/D.points.length),[0,0,0]):[0,0,0];
const shown=new Set(D.objects.map(o=>o.id));
for(const o of D.objects){const label=document.createElement('label'),cb=document.createElement('input');cb.type='checkbox';cb.checked=true;cb.onchange=()=>{cb.checked?shown.add(o.id):shown.delete(o.id);draw()};label.append(cb,document.createTextNode(' #'+o.id+' '+o.label+' · '+o.points+' points'));document.querySelector('#objects').append(label)}
// Exported points follow object order; subsampling preserves their global indices.
const ranges=[];let end=0;for(const o of D.objects){ranges.push([end,end+=o.points,o.id])}
const step=Math.max(1,Math.ceil(end/100000));
function project(p){let x=p[0]-center[0],y=p[1]-center[1],z=p[2]-center[2];let xx=Math.cos(yaw)*x+Math.sin(yaw)*z,zz=-Math.sin(yaw)*x+Math.cos(yaw)*z;return [canvas.width/2+pan[0]+zoom*xx,canvas.height/2+pan[1]+zoom*(Math.cos(pitch)*y-Math.sin(pitch)*zz),Math.sin(pitch)*y+Math.cos(pitch)*zz]}
function line(a,b,color){a=project(a);b=project(b);ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke()}
function draw(){canvas.width=innerWidth;canvas.height=innerHeight;ctx.fillStyle='#11151b';ctx.fillRect(0,0,canvas.width,canvas.height);line([0,0,0],[.3,0,0],'#ff6464');line([0,0,0],[0,.3,0],'#64ff64');line([0,0,0],[0,0,.3],'#6496ff');
let points=[];let ri=0;for(let i=0;i<D.points.length;i++){while(ri+1<ranges.length&&i*step>=ranges[ri][1])ri++;if(ranges.length&&!shown.has(ranges[ri][2]))continue;points.push([project(D.points[i]),D.colors[i]])}points.sort((a,b)=>b[0][2]-a[0][2]);for(const [p,c] of points){ctx.fillStyle='rgb('+c.join(',')+')';ctx.fillRect(p[0],p[1],2,2)}
if(document.querySelector('#trail').checked){let prev=null;for(const t of D.trajectory){const p=t.T_world_camera.slice(0,3).map(r=>r[3]);if(prev)line(prev,p,'#ffcf57');prev=p}}
ctx.font='14px system-ui';ctx.fillStyle='white';for(const o of D.objects){if(!shown.has(o.id))continue;const p=project(o.center_m);ctx.fillText('#'+o.id+' '+o.label,p[0]+5,p[1]-5)}}
canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};
canvas.onpointerup=canvas.onpointercancel=()=>drag=null;
canvas.onpointermove=e=>{if(!drag)return;let dx=e.clientX-drag[0],dy=e.clientY-drag[1];if(e.shiftKey){pan[0]+=dx;pan[1]+=dy}else{yaw+=dx*.006;pitch+=dy*.006}drag=[e.clientX,e.clientY];draw()};
canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(5,Math.min(5000,zoom*Math.exp(-e.deltaY*.001)));draw()};
document.querySelector('#reset').onclick=()=>{yaw=-.35;pitch=-.2;zoom=220;pan=[0,0];draw()};document.querySelector('#trail').onchange=draw;onresize=draw;draw();</script>''')
