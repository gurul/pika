"""Make a self-contained playback of the actual simulator traces."""
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1] / "build/active-roam-simulation"
data = [json.loads((root / f"{name}.json").read_text())
        for name in ("hallway", "offset_obstacle", "noisy_obstacle")]
page = '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Active sonar — simulation replay</title>
<style>
body{margin:0;background:#10171f;color:#e8edf1;font:16px system-ui}main{max-width:1060px;margin:auto;padding:28px}
h1{font-size:28px;margin:0 0 8px}p{color:#afbdc9;line-height:1.5}canvas{width:100%;background:#16212c;border:1px solid #3a4c5a;border-radius:10px}
.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:18px 0}select,button{font:inherit;padding:10px;border-radius:6px;background:#23374b;color:white;border:1px solid #56718a}input{flex:1;min-width:160px}#state{font-variant-numeric:tabular-nums;color:#65dfce}.legend{font-size:14px}strong{color:white}
</style><main><h1>Look first. Then steer.</h1>
<p>This replays the implemented controller in a simplified simulation. <strong>It is not a recording of your car.</strong> The blue fan shows the commanded sonar direction; the green trail shows the path.</p>
<div class="controls"><label for="scenario">Scenario</label><select id="scenario"></select><button id="play">Play</button><input id="scrub" aria-label="Simulation time" type="range" min="0" value="0"><output id="time"></output></div>
<canvas id="map" width="1000" height="520" aria-label="Simulated car path, obstacles and sonar direction"></canvas>
<p id="state"></p><p id="stats"></p>
<p class="legend">Grey: obstacles · Green: path · Blue: sonar cone · Orange: selected direction. The car slows or stops when it lacks a recent view. A side scan can continue while the wheels are stopped.</p>
<p>Next physical checks: servo settling, steering response and braking on a charged battery. The faster-wheel stress case still gets stuck; the controller is not yet seamless.</p>
</main><script>
const runs=__DATA__,sel=document.querySelector('#scenario'),scrub=document.querySelector('#scrub'),ctx=document.querySelector('canvas').getContext('2d');
let running=false,index=0,last=0;
runs.forEach((r,i)=>{let o=document.createElement('option');o.value=i;o.textContent=r.stats.name.replaceAll('_',' ');sel.append(o)});
const X=x=>30+x*1.56,Y=y=>490-y*1.5;
function draw(){const r=runs[+sel.value],t=r.trace[index];ctx.clearRect(0,0,1000,520);ctx.fillStyle='#85909a';r.obstacles.forEach(([x0,x1,y0,y1])=>ctx.fillRect(X(x0),Y(y1),(x1-x0)*1.56,(y1-y0)*1.5));
ctx.strokeStyle='#68dfb4';ctx.lineWidth=3;ctx.beginPath();r.trace.slice(0,index+1).forEach((q,i)=>i?ctx.lineTo(X(q.x),Y(q.y)):ctx.moveTo(X(q.x),Y(q.y)));ctx.stroke();
let h=t.yaw*Math.PI/180,b=h+(t.look-90)*Math.PI/180;ctx.save();ctx.translate(X(t.x),Y(t.y));ctx.scale(1,-1);ctx.fillStyle='#48b8ed30';ctx.beginPath();ctx.moveTo(0,0);ctx.arc(0,0,140,b-Math.PI/12,b+Math.PI/12);ctx.closePath();ctx.fill();
ctx.strokeStyle='#faad57';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(65*Math.cos(h+t.heading*Math.PI/180),65*Math.sin(h+t.heading*Math.PI/180));ctx.stroke();ctx.rotate(h);ctx.fillStyle='#e9f4fa';ctx.fillRect(-16,-12,32,24);ctx.fillStyle='#35aaf0';ctx.fillRect(12,-8,7,16);ctx.restore();
document.querySelector('#time').textContent=t.t.toFixed(2)+' s';document.querySelector('#state').textContent=t.reason.replaceAll('_',' ')+'  ·  sonar '+t.look+'°  ·  wheels '+Math.round(t.left)+' / '+Math.round(t.right)+' PWM';
const s=r.stats;document.querySelector('#stats').textContent='Complete run: '+s.contacts+' contacts · '+(s.distance_cm/100).toFixed(2)+' m travelled · '+s.stops+' transitions from driving to a stopped/recovery state';scrub.value=index;}
function reset(){index=0;scrub.max=runs[+sel.value].trace.length-1;draw()}sel.onchange=reset;scrub.oninput=()=>{index=+scrub.value;draw()};document.querySelector('#play').onclick=()=>{running=!running;document.querySelector('#play').textContent=running?'Pause':'Play'};
function animate(ms){if(running&&ms-last>=50){index=(index+1)%runs[+sel.value].trace.length;draw();last=ms}requestAnimationFrame(animate)}reset();requestAnimationFrame(animate);
</script></html>'''
(root / "replay.html").write_text(page.replace("__DATA__", json.dumps(data).replace("</", "<\\/")))
print(root / "replay.html")
