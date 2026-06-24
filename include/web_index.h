#pragma once
#include <pgmspace.h>

// Single-page tuning UI. Camera preview is an MJPEG stream served on port 81.
// All tuning is done over /set and live state is polled from /status.
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>XIAO Underwater Motion Detector</title>
<style>
  :root { --bg:#0b1622; --panel:#13212f; --line:#1f3346; --txt:#e8f0f7; --mut:#8aa0b3; --accent:#27c2a0; --hot:#ff5d5d; }
  * { box-sizing:border-box; }
  body { margin:0; font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; background:var(--bg); color:var(--txt); }
  header { padding:14px 18px; border-bottom:1px solid var(--line); display:flex; align-items:center; gap:12px; }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  .dot { width:12px; height:12px; border-radius:50%; background:#3a4d5e; transition:background .15s; }
  .dot.on { background:var(--hot); box-shadow:0 0 12px var(--hot); }
  main { display:grid; grid-template-columns: minmax(320px,1fr) 360px; gap:16px; padding:16px; }
  @media (max-width: 820px){ main { grid-template-columns:1fr; } }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:12px; padding:14px; }
  .card h2 { font-size:12px; text-transform:uppercase; letter-spacing:.08em; color:var(--mut); margin:0 0 12px; }
  .preview { display:flex; flex-direction:column; gap:10px; }
  #cam { width:100%; border-radius:8px; background:#000; aspect-ratio:4/3; object-fit:contain; }
  .stat-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .stat { background:#0e1a26; border:1px solid var(--line); border-radius:8px; padding:8px 10px; }
  .stat .k { font-size:10px; color:var(--mut); text-transform:uppercase; letter-spacing:.05em; }
  .stat .v { font-size:18px; font-weight:600; margin-top:2px; }
  .bar { height:8px; background:#0e1a26; border-radius:5px; overflow:hidden; margin-top:6px; border:1px solid var(--line); }
  .bar > div { height:100%; width:0%; background:var(--accent); transition:width .1s; }
  .row { margin-bottom:16px; }
  .row label { display:flex; justify-content:space-between; font-size:13px; margin-bottom:6px; }
  .row label b { color:var(--accent); font-variant-numeric:tabular-nums; }
  input[type=range]{ width:100%; accent-color:var(--accent); }
  .btns { display:flex; gap:10px; margin-top:6px; }
  button { flex:1; padding:10px; border-radius:8px; border:1px solid var(--line); background:#0e1a26; color:var(--txt); font-size:13px; cursor:pointer; }
  button.primary { background:var(--accent); color:#04231d; border-color:var(--accent); font-weight:600; }
  button:active { transform:translateY(1px); }
  .hint { font-size:11px; color:var(--mut); margin-top:8px; line-height:1.5; }
  .pill { font-size:11px; padding:2px 8px; border-radius:20px; border:1px solid var(--line); color:var(--mut); }
  .pill.hi { color:var(--accent); border-color:var(--accent); }
</style>
</head>
<body>
<header>
  <span id="motiondot" class="dot"></span>
  <h1>XIAO Underwater Motion Detector</h1>
  <span id="conn" class="pill">connecting...</span>
</header>
<main>
  <section class="card preview">
    <h2>Camera preview</h2>
    <img id="cam" src="" alt="camera stream">
    <div class="stat-grid">
      <div class="stat"><div class="k">Motion</div><div class="v" id="s_motion">--</div></div>
      <div class="stat"><div class="k">Changed pixels</div><div class="v" id="s_changed">--%</div>
        <div class="bar"><div id="s_changed_bar"></div></div></div>
      <div class="stat"><div class="k">Digital out (D0)</div><div class="v" id="s_dout">--</div></div>
      <div class="stat"><div class="k">Servo PWM (D1)</div><div class="v" id="s_servo">--</div></div>
    </div>
    <div class="hint">Servo output holds <b>2000&micro;s</b> while active, drops to <b>1000&micro;s</b> when idle.
      Digital output mirrors the active state. "Active" = motion + hysteresis hold time.</div>
  </section>

  <section class="card">
    <h2>Motion tuning</h2>
    <div class="row">
      <label>Pixel change threshold <b id="v_pix">--</b></label>
      <input type="range" id="pix" min="2" max="120" step="1">
      <div class="hint">How much a single pixel must change (0-255) to count as "changed". Lower = more sensitive.</div>
    </div>
    <div class="row">
      <label>Min changed area <b id="v_area">--%</b></label>
      <input type="range" id="area" min="1" max="400" step="1">
      <div class="hint">Percent of sampled pixels that must change to trigger motion (0.1% steps).</div>
    </div>
    <div class="row">
      <label>Hysteresis hold <b id="v_hyst">--</b></label>
      <input type="range" id="hyst" min="0" max="30000" step="100">
      <div class="hint">Outputs stay active this long after the last motion before releasing.</div>
    </div>
    <div class="row">
      <label>Sample interval <b id="v_iv">--</b></label>
      <input type="range" id="iv" min="33" max="1000" step="1">
      <div class="hint">Time between motion-detection frames. Higher = slower but lighter.</div>
    </div>
    <div class="btns">
      <button class="primary" id="save">Save to flash</button>
      <button id="reset">Reset defaults</button>
    </div>
    <div class="hint" id="savemsg"></div>
  </section>
</main>

<script>
const $ = id => document.getElementById(id);
const sliders = { pix:'pix', area:'area', hyst:'hyst', iv:'iv' };
let busy = false;

function fmtMs(ms){ return ms >= 1000 ? (ms/1000).toFixed(1)+' s' : ms+' ms'; }

function paintSliderLabels(){
  $('v_pix').textContent = $('pix').value;
  $('v_area').textContent = ($('area').value/10).toFixed(1)+'%';
  $('v_hyst').textContent = fmtMs(+$('hyst').value);
  $('v_iv').textContent = $('iv').value+' ms';
}

async function loadSettings(){
  const r = await fetch('/settings'); const s = await r.json();
  $('pix').value = s.pixelThreshold;
  $('area').value = s.minChangedPermille;
  $('hyst').value = s.hysteresisMs;
  $('iv').value = s.intervalMs;
  paintSliderLabels();
}

async function pushSetting(key, value){
  await fetch('/set?'+key+'='+value);
}

Object.values(sliders).forEach(id => {
  const el = $(id);
  el.addEventListener('input', paintSliderLabels);
  el.addEventListener('change', async () => {
    const map = { pix:'pixelThreshold', area:'minChangedPermille', hyst:'hysteresisMs', iv:'intervalMs' };
    await pushSetting(map[id], el.value);
  });
});

$('save').onclick = async () => {
  await fetch('/save'); $('savemsg').textContent = 'Saved to flash at '+new Date().toLocaleTimeString();
};
$('reset').onclick = async () => {
  await fetch('/reset'); await loadSettings(); $('savemsg').textContent = 'Reset to defaults.';
};

async function poll(){
  try {
    const r = await fetch('/status'); const s = await r.json();
    $('conn').textContent = 'online'; $('conn').classList.add('hi');
    $('motiondot').classList.toggle('on', s.active);
    $('s_motion').textContent = s.motion ? 'YES' : (s.active ? 'hold' : 'no');
    $('s_changed').textContent = (s.changedPermille/10).toFixed(1)+'%';
    $('s_changed_bar').style.width = Math.min(100, s.changedPermille/10)+'%';
    $('s_dout').textContent = s.active ? 'HIGH' : 'LOW';
    $('s_servo').textContent = (s.active ? 2000 : 1000)+' us';
  } catch(e){
    $('conn').textContent = 'offline'; $('conn').classList.remove('hi');
  }
}

function startStream(){
  // MJPEG stream lives on port 81 of the same host.
  $('cam').src = 'http://'+location.hostname+':81/stream';
}

loadSettings().then(startStream);
setInterval(poll, 400);
</script>
</body>
</html>
)HTML";
