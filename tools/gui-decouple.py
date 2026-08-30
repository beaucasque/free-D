#!/usr/bin/env python3
"""
gui-decouple.py — Instrument de controle du decouplage camera / objectif.

MEME MESURE QUE test-decouple.py, MAIS EN CONTINU

Bagues bloquees, camera qui bouge : le zoom et le focus ne doivent pas
broncher. Deux chaines calculees en parallele sur les memes echantillons —
"naif" (dernier q_camera connu) et "aligne" (q_camera interpole par slerp a
l'horodatage de l'echantillon objectif) — tracees SUR LE MEME GRATICULE.

C'est tout l'interet du GUI : quand tu vrilles la camera, la trace ambre
decolle et la trace cyan reste sur la ligne. Tu n'as pas besoin de lire un
chiffre pour savoir si ca marche, et surtout tu peux bouger la camera d'une
main en regardant l'ecran, ce qu'un rapport en fin de phase ne permet pas.

La bande claire derriere les traces est la tolerance : 0,3 % de la course.
Rester dedans, c'est le critere. La bande plus large a 1 % est la limite du
passable.

AUCUNE DEPENDANCE GUI — serveur HTTP de la bibliotheque standard, page servie
en local, traces dessinees au canvas. Rien a installer, rien qui vienne d'un
CDN : la station n'a pas besoin d'acces reseau.

USAGE

    ./gui-decouple.py                  # puis ouvrir http://127.0.0.1:8410
    ./gui-decouple.py --demo           # sans materiel, pour voir l'interface
    ./gui-decouple.py --host 0.0.0.0   # accessible depuis le reseau

Depuis le Mac, plutot qu'exposer le port :

    ssh -L 8410:localhost:8410 unreal

BLOQUER LES BAGUES AU RUBAN avant de commencer, sauf pour la phase 'bagues'.
"""

import argparse
import importlib.util
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "bridge"))

import lensaxis  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "test_decouple", os.path.join(HERE, "test-decouple.py"))
_td = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_td)

DEFAULT_CONFIG = os.path.join(HERE, "..", "bridge", "axes.json")


# --------------------------------------------------------------------- moteur


class Runner:
    """Draine libsurvive en continu et tient l'etat courant.

    Un seul thread parle a pysurvive. Le serveur HTTP ne fait que lire un
    instantane sous verrou : pas de contention, pas de risque de double
    consommation de la file.
    """

    def __init__(self, cfg, demo=False):
        self.cfg = cfg
        self.demo = demo
        self.tap = _td.Tap(cfg)
        self.lock = threading.Lock()
        self.phase = "libre"
        self.phase_started = None
        self.done_phases = []
        self.ref = None
        self.live = {a: {"naif": 0.0, "aligne": 0.0} for a in cfg["axes"]}
        self.cam_rate = 0.0
        self.stop = threading.Event()
        self.ctx = None
        if not demo:
            try:
                import pysurvive
            except ImportError:
                sys.exit("pysurvive absent. Voir bridge/requirements.txt")
            self.ctx = pysurvive.SimpleContext(sys.argv[:1])

    # -- boucle ------------------------------------------------------------

    def loop(self):
        n = 0
        while not self.stop.is_set():
            if self.demo:
                self._demo_step(time.monotonic(), n)
                n += 1
            else:
                self.tap.drain(self.ctx)
            with self.lock:
                self.tap.resolve(self.phase)
                self._refresh()
            time.sleep(0.004)

    def _demo_step(self, t, n):
        """Donnees fabriquees : roulis oscillant, bagues immobiles sauf en
        phase 'bagues'. Le decalage de 3 ms entre les deux trackers est ce
        qui fait decoller la trace naive."""
        def q(axis, a):
            v = np.array(axis, float)
            v /= np.linalg.norm(v)
            s = math.sin(a / 2.0)
            return lensaxis.q_norm((math.cos(a / 2.0),
                                    v[0] * s, v[1] * s, v[2] * s))

        moving = self.phase in ("panoramique", "tilt", "roulis", "travelling")
        amp = {"panoramique": 40.0, "tilt": 25.0,
               "roulis": 55.0, "travelling": 0.0}.get(self.phase, 0.0)
        # Un pignon de follow focus tourne autour d'un axe parallele a l'axe
        # optique : le roulis lui est colineaire et fuit, le panoramique et
        # le tilt lui sont orthogonaux et fuient peu. La demo le reproduit.
        ax = {"panoramique": [0, 0, 1], "tilt": [0, 1, 0],
              "roulis": [1, 0, 0]}.get(self.phase, [0, 0, 1])

        def cam(tt):
            return math.radians(amp) * math.sin(2.0 * math.pi * 0.7 * tt) \
                if moving else 0.0

        q_cam = q(ax, cam(t))
        latest = self.tap.hist.latest()
        q_naive = latest[0] if self.tap.hist.t else None
        self.tap.hist.push(t, q_cam, (0.0, 0.0, 1.4))
        if n < 4:
            return
        tl = t - 0.003          # le tracker objectif est horodate en retard
        for name, cal in self.cfg["axes"].items():
            bague = math.radians(120.0)
            if self.phase == "bagues":
                bague += math.radians(cal["span_deg"] * 0.45) * \
                    math.sin(2.0 * math.pi * 0.2 * t)
            q_lens = lensaxis.q_mul(q(ax, cam(tl)),
                                    q(cal["axis"], bague))
            self.tap.pending.append((tl, name, q_lens, (0.12, 0.0, 1.4),
                                     q_naive))
            self.tap.st[name]["last_t"] = tl

    def _refresh(self):
        self.cam_rate = math.degrees(self.tap.hist.rate())
        for axis in self.cfg["axes"]:
            rows = [r for r in self.tap.rows[-400:] if r["axis"] == axis]
            if rows:
                self.live[axis] = {"naif": math.degrees(rows[-1]["theta_naif"]),
                                   "aligne": math.degrees(rows[-1]["theta_aligne"])}

    # -- commandes ---------------------------------------------------------

    def start_phase(self, name):
        with self.lock:
            self.phase = name
            self.phase_started = time.monotonic()

    def end_phase(self):
        with self.lock:
            if self.phase not in ("libre", None):
                if self.phase not in self.done_phases:
                    self.done_phases.append(self.phase)
                if self.phase == "repos":
                    self.ref = {a: _td.reference(self.tap.rows, a)
                                for a in self.cfg["axes"]}
            self.phase = "libre"
            self.phase_started = None

    def reset(self):
        with self.lock:
            self.tap = _td.Tap(self.cfg)
            self.done_phases = []
            self.ref = None
            self.phase = "libre"
            self.phase_started = None

    # -- lecture -----------------------------------------------------------

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            out = {"phase": self.phase, "done": list(self.done_phases),
                   "elapsed": (now - self.phase_started)
                   if self.phase_started else 0.0,
                   "cam_rate": self.cam_rate, "axes": {},
                   "ref_ok": self.ref is not None}

            tot = self.tap.n_exact + self.tap.n_defer + self.tap.n_stale
            out["exact_pct"] = 100.0 * self.tap.n_exact / max(1, tot)
            out["cam_gap_ms"] = self.tap.cam_gap * 1000.0
            out["cam_seen"] = bool(self.tap.hist.t)

            for axis, cal in self.cfg["axes"].items():
                st = self.tap.st[axis]
                r = self.ref.get(axis) if self.ref else None
                dn = self.live[axis]["naif"] - (r["naif"] if r else 0.0)
                da = self.live[axis]["aligne"] - (r["aligne"] if r else 0.0)
                rows = [x for x in self.tap.rows if x["axis"] == axis]
                # Crete maintenue sur la phase en cours. Un operateur qui
                # vrille la camera d'une main ne peut pas lire une valeur
                # instantanee : c'est le pic qui juge la phase, comme sur un
                # crete-metre.
                # Fenetre de garde : la demi-seconde qui suit le demarrage
                # d'une phase est exclue. L'operateur a besoin de ce temps
                # pour reprendre la camera, et la crete retiendrait sinon ce
                # seul geste au lieu du mouvement demande.
                t_ok = (self.phase_started or 0.0) + 0.5
                ph = [x for x in rows
                      if x["phase"] == self.phase and x["t"] >= t_ok] \
                    if self.phase != "libre" else rows[-2000:]
                pn = pa = 0.0
                if ph and r:
                    pn = float(np.max(np.abs(
                        np.degrees([x["theta_naif"] for x in ph]) - r["naif"])))
                    pa = float(np.max(np.abs(
                        np.degrees([x["theta_aligne"] for x in ph]) - r["aligne"])))
                skew = drift = 0.0
                if len(rows) > 30:
                    tail = rows[-2000:]
                    rate = np.degrees([x["cam_rate"] for x in tail])
                    dev = np.abs(np.degrees([x["theta_aligne"] for x in tail])
                                 - (r["aligne"] if r else 0.0))
                    den = float(np.sum(rate ** 2))
                    skew = (float(np.sum(rate * dev)) / den * 1000.0) \
                        if den > 1e-9 else 0.0
                    d = [x["dist"] for x in tail]
                    drift = (max(d) - min(d)) * 1000.0
                out["axes"][axis] = {
                    "naif": dn, "aligne": da,
                    "naif_peak": pn, "aligne_peak": pa,
                    "span": cal["span_deg"],
                    "counts": int(pa / cal["span_deg"] * 65535),
                    "pct": pa / cal["span_deg"] * 100.0,
                    "skew_ms": skew, "drift_mm": drift,
                    "seen": bool(st["last_t"]),
                    "gap_ms": st["max_gap"] * 1000.0,
                    "multiturn": cal["span_deg"] >= 355.0,
                    "dropout": st["acc_a"].dropout(),
                }
            return out


# ------------------------------------------------------------------ interface

PAGE = r"""<!DOCTYPE html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Découplage caméra / objectif</title>
<style>
:root{
  --panel:#1b1e24;        /* graphite, pas noir : moins fatigant en studio */
  --panel-2:#22262e;
  --rule:#31363f;
  --ink:#e8e6e1;
  --ink-dim:#8b929e;
  --naif:#e2a03f;         /* ambre : la chaîne non corrigée */
  --aligne:#4fc3d9;       /* cyan : la chaîne corrigée */
  --cam:#c77dbb;          /* magenta : vitesse caméra */
  --ok:#7fbf6a;
  --warn:#e2a03f;
  --bad:#e05c5c;
}
*{box-sizing:border-box}
html,body{margin:0;height:100%}
body{
  background:var(--panel);color:var(--ink);
  font-family:"Ubuntu","DejaVu Sans",system-ui,sans-serif;
  font-size:14px;display:flex;flex-direction:column;
}
.mono{font-family:"Ubuntu Mono","DejaVu Sans Mono",monospace;
      font-variant-numeric:tabular-nums}
.eyebrow{font-family:"Ubuntu Condensed","Ubuntu","DejaVu Sans Condensed",sans-serif;
  text-transform:uppercase;letter-spacing:.14em;font-size:11px;color:var(--ink-dim)}

header{display:flex;align-items:baseline;gap:20px;padding:14px 18px;
  border-bottom:1px solid var(--rule);flex-wrap:wrap}
header h1{margin:0;font-family:"Ubuntu Condensed","Ubuntu",sans-serif;
  font-size:19px;font-weight:500;letter-spacing:.03em}
.lamps{display:flex;gap:14px;margin-left:auto;flex-wrap:wrap}
.lamp{display:flex;align-items:center;gap:6px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--bad);
  box-shadow:0 0 7px currentColor;color:var(--bad)}
.dot.ok{background:var(--ok);color:var(--ok)}
.dot.warn{background:var(--warn);color:var(--warn)}

main{flex:1;display:grid;grid-template-columns:236px 1fr;min-height:0}
@media(max-width:820px){main{grid-template-columns:1fr}}

aside{border-right:1px solid var(--rule);padding:16px 14px;overflow:auto;
  background:var(--panel-2)}
ol{list-style:none;margin:10px 0 0;padding:0;counter-reset:p}
ol li{counter-increment:p;display:grid;grid-template-columns:22px 1fr;
  gap:9px;padding:9px 8px;border-radius:3px;cursor:pointer;align-items:start}
ol li:hover{background:#2a2f38}
ol li::before{content:counter(p);font-family:"Ubuntu Mono",monospace;
  color:var(--ink-dim);font-size:12px;padding-top:2px}
ol li.on{background:#2d3a42;outline:1px solid var(--aligne)}
ol li.done::before{content:"✓";color:var(--ok)}
ol li b{font-weight:500;display:block}
ol li.key b{color:var(--naif)}
ol li small{color:var(--ink-dim);font-size:11.5px;line-height:1.35;display:block}
.btns{display:flex;gap:8px;margin-top:16px}
button{flex:1;background:#2d3a42;color:var(--ink);border:1px solid var(--rule);
  border-radius:3px;padding:9px;font-family:"Ubuntu Condensed",sans-serif;
  text-transform:uppercase;letter-spacing:.09em;font-size:12px;cursor:pointer}
button:hover{background:#37454e}
button:focus-visible{outline:2px solid var(--aligne);outline-offset:2px}
button.stop{background:#432d2d;border-color:#5a3a3a}

section.scopes{padding:14px 18px;overflow:auto;min-width:0}
.scope{margin-bottom:16px}
.scope-head{display:flex;align-items:baseline;gap:14px;margin-bottom:5px;
  flex-wrap:wrap}
.scope-head .val{margin-left:auto;display:flex;gap:18px;align-items:baseline}
.val b{font-size:22px;font-weight:400}
.val b.naif{color:var(--naif)}
.val b.aligne{color:var(--aligne)}
canvas{width:100%;display:block;background:#171a1f;border:1px solid var(--rule);
  border-radius:2px}

.strip{display:flex;gap:26px;padding:11px 18px;border-top:1px solid var(--rule);
  flex-wrap:wrap;background:var(--panel-2)}
.strip div span{display:block}
.strip .n{font-size:17px}
.hint{padding:0 18px 14px;color:var(--ink-dim);font-size:12.5px;max-width:70ch;
  line-height:1.5}
</style>

<header>
  <h1>Découplage caméra / objectif</h1>
  <span class="eyebrow" id="phase">au repos</span>
  <div class="lamps">
    <div class="lamp"><i class="dot" id="l-cam"></i><span class="eyebrow">caméra</span></div>
    <div class="lamp"><i class="dot" id="l-zoom"></i><span class="eyebrow">zoom</span></div>
    <div class="lamp"><i class="dot" id="l-focus"></i><span class="eyebrow">focus</span></div>
  </div>
</header>

<main>
  <aside>
    <div class="eyebrow">Ordre des phases</div>
    <ol id="phases"></ol>
    <div class="btns">
      <button id="stop" class="stop">Arrêter</button>
      <button id="reset">Repartir</button>
    </div>
    <p class="hint" style="padding:14px 0 0">
      Bloque les bagues au ruban. La phase <b>repos</b> établit la référence :
      fais-la en premier, sinon les écarts n'ont pas d'origine.
    </p>
  </aside>

  <section class="scopes" id="scopes"></section>
</main>

<div class="strip">
  <div><span class="eyebrow">Fuite</span><span class="n mono" id="s-leak">—</span></div>
  <div><span class="eyebrow">Résidu temporel</span><span class="n mono" id="s-skew">—</span></div>
  <div><span class="eyebrow">Échantillons exploitables</span><span class="n mono" id="s-exact">—</span></div>
  <div><span class="eyebrow">Trou max caméra</span><span class="n mono" id="s-gap">—</span></div>
  <div><span class="eyebrow">Dérive du montage</span><span class="n mono" id="s-drift">—</span></div>
</div>

<script>
const N = 900;               // points gardés par trace
const scopes = {};           // axe -> {buf, canvas, ...}
let cfg = null, camBuf = ring(N);

function ring(n){ return {a:new Float32Array(n), i:0, n:0}; }
function push(r,v){ r.a[r.i]=v; r.i=(r.i+1)%r.a.length; if(r.n<r.a.length)r.n++; }
function at(r,k){ return r.a[(r.i - r.n + k + r.a.length) % r.a.length]; }

function el(t,c,x){ const e=document.createElement(t); if(c)e.className=c;
  if(x!==undefined)e.textContent=x; return e; }

fetch("config").then(r=>r.json()).then(c=>{ cfg=c; build(); tick(); });

function build(){
  const ol = document.getElementById("phases");
  cfg.phases.forEach(p=>{
    const li = el("li"); li.dataset.name = p.name;
    if(p.name === "roulis") li.className = "key";
    li.appendChild(el("b", null, p.name));
    li.appendChild(el("small", null, p.how));
    li.onclick = ()=> fetch("cmd?start="+encodeURIComponent(p.name));
    ol.appendChild(li);
  });
  document.getElementById("stop").onclick = ()=> fetch("cmd?stop=1");
  document.getElementById("reset").onclick = ()=> {
    fetch("cmd?reset=1"); Object.values(scopes).forEach(s=>s.buf={naif:ring(N),aligne:ring(N)});
    camBuf = ring(N);
  };

  const host = document.getElementById("scopes");
  for(const axis of cfg.axes){
    const d = el("div","scope");
    const h = el("div","scope-head");
    h.appendChild(el("span","eyebrow", axis + " — écart par rapport au repos"));
    const v = el("div","val");
    const bn = el("b","mono naif","—"), ba = el("b","mono aligne","—");
    v.appendChild(el("span","eyebrow","naïf")); v.appendChild(bn);
    v.appendChild(el("span","eyebrow","aligné")); v.appendChild(ba);
    const pk = el("span","eyebrow mono","crête —"); pk.style.marginLeft="14px";
    v.appendChild(pk);
    h.appendChild(v); d.appendChild(h);
    const c = el("canvas"); c.height = 150; d.appendChild(c);
    host.appendChild(d);
    scopes[axis] = {canvas:c, bn:bn, ba:ba, pk:pk,
                    buf:{naif:ring(N), aligne:ring(N)}, span:cfg.span[axis]};
  }
  const d = el("div","scope");
  const h = el("div","scope-head");
  h.appendChild(el("span","eyebrow","vitesse angulaire caméra"));
  const v = el("div","val"); const bc = el("b","mono","—");
  bc.style.color = "var(--cam)"; v.appendChild(bc); h.appendChild(v);
  d.appendChild(h);
  const c = el("canvas"); c.height = 92; d.appendChild(c);
  host.appendChild(d);
  scopes.__cam = {canvas:c, b:bc};
}

const ev = new EventSource("stream");
ev.onmessage = e=>{
  const s = JSON.parse(e.data);
  document.getElementById("phase").textContent =
    s.phase === "libre" ? "au repos" :
    s.phase + " · " + s.elapsed.toFixed(0) + " s";

  lamp("l-cam", s.cam_seen, s.cam_gap_ms > 200);
  push(camBuf, s.cam_rate);
  scopes.__cam.b.textContent = s.cam_rate.toFixed(0) + " °/s";

  let leak = 0, skew = 0, drift = 0;
  for(const [axis, a] of Object.entries(s.axes)){
    lamp("l-"+axis, a.seen, a.gap_ms > 200 || a.dropout);
    const sc = scopes[axis]; if(!sc) continue;
    push(sc.buf.naif, a.naif); push(sc.buf.aligne, a.aligne);
    sc.bn.textContent = fmt(a.naif); sc.ba.textContent = fmt(a.aligne);
    sc.pk.textContent = "crête " + a.naif_peak.toFixed(2) + "° / "
                      + a.aligne_peak.toFixed(2) + "°";
    leak = Math.max(leak, a.counts); skew = Math.max(skew, a.skew_ms);
    drift = Math.max(drift, a.drift_mm);
  }
  set("s-leak", leak + " counts");
  set("s-skew", skew.toFixed(1) + " ms");
  set("s-exact", s.exact_pct.toFixed(1) + " %");
  set("s-gap", s.cam_gap_ms.toFixed(0) + " ms");
  set("s-drift", drift.toFixed(1) + " mm");

  document.querySelectorAll("#phases li").forEach(li=>{
    li.classList.toggle("on", li.dataset.name === s.phase);
    li.classList.toggle("done", s.done.includes(li.dataset.name));
  });
  if(!s.ref_ok) document.getElementById("phase").textContent += "  (référence non établie)";
};

function fmt(v){ return (v>=0?"+":"") + v.toFixed(3) + "°"; }
function set(id,v){ document.getElementById(id).textContent = v; }
function lamp(id, seen, warn){
  const e = document.getElementById(id); if(!e) return;
  e.className = "dot" + (!seen ? "" : (warn ? " warn" : " ok"));
}

function tick(){
  for(const [axis, sc] of Object.entries(scopes)){
    if(axis === "__cam") drawCam(sc); else drawAxis(sc);
  }
  requestAnimationFrame(tick);
}

function prep(cv){
  const r = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.height;
  if(cv.width !== w*r || cv._h !== h*r){ cv.width = w*r; cv.height = h*r; cv._h = h*r; }
  const g = cv.getContext("2d"); g.setTransform(r,0,0,r,0,0);
  g.clearRect(0,0,w,h);
  return [g,w,h];
}

function drawAxis(sc){
  const [g,w,h] = prep(sc.canvas);
  // L'échelle est ±2 % de la course : la tolérance occupe une fraction
  // lisible de la hauteur, et un dépassement franc sort du cadre.
  const full = sc.span * 0.02, mid = h/2, k = (h/2 - 6) / full;

  // bandes de tolérance : 0,3 % = bon, 1 % = passable
  band(g, w, mid, sc.span*0.003*k, "rgba(127,191,106,.10)");
  band(g, w, mid, sc.span*0.010*k, "rgba(226,160,63,.07)");

  g.strokeStyle="#2b3038"; g.lineWidth=1;
  for(let i=1;i<4;i++){ const y=(h/4)*i; g.beginPath();
    g.moveTo(0,y+.5); g.lineTo(w,y+.5); g.stroke(); }
  g.strokeStyle="#4a515c"; g.beginPath();
  g.moveTo(0,mid+.5); g.lineTo(w,mid+.5); g.stroke();

  trace(g, sc.buf.naif, w, mid, k, "#e2a03f", 1.2);
  trace(g, sc.buf.aligne, w, mid, k, "#4fc3d9", 1.8);

  g.fillStyle="#8b929e"; g.font='11px "Ubuntu Mono","DejaVu Sans Mono",monospace';
  g.fillText("±"+(sc.span*0.003).toFixed(2)+"° tolérance", 7, 14);
}

function band(g,w,mid,half,fill){
  g.fillStyle = fill; g.fillRect(0, mid-half, w, half*2);
}

function drawCam(sc){
  const [g,w,h] = prep(sc.canvas);
  let mx = 60; for(let i=0;i<camBuf.n;i++) mx = Math.max(mx, at(camBuf,i));
  const k = (h-10)/mx;
  g.strokeStyle="#2b3038"; g.beginPath();
  g.moveTo(0,h-.5); g.lineTo(w,h-.5); g.stroke();
  g.strokeStyle="#c77dbb"; g.lineWidth=1.4; g.beginPath();
  const n = camBuf.n, step = w/Math.max(1,N-1);
  for(let i=0;i<n;i++){
    const x = w - (n-1-i)*step, y = h - at(camBuf,i)*k;
    i ? g.lineTo(x,y) : g.moveTo(x,y);
  }
  g.stroke();
  g.fillStyle="#8b929e"; g.font='11px "Ubuntu Mono","DejaVu Sans Mono",monospace';
  g.fillText(mx.toFixed(0)+" °/s", 7, 14);
}

function trace(g, r, w, mid, k, color, lw){
  if(!r.n) return;
  g.strokeStyle=color; g.lineWidth=lw; g.beginPath();
  const step = w/Math.max(1,N-1);
  for(let i=0;i<r.n;i++){
    const x = w - (r.n-1-i)*step;
    const y = Math.max(1, Math.min(mid*2-1, mid - at(r,i)*k));
    i ? g.lineTo(x,y) : g.moveTo(x,y);
  }
  g.stroke();
}
</script>
"""


class Handler(BaseHTTPRequestHandler):
    runner = None
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        path, _, query = self.path.partition("?")
        q = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
        r = Handler.runner

        if path in ("/", "/index.html"):
            return self._send(PAGE, "text/html; charset=utf-8")

        if path == "/config":
            return self._send(json.dumps({
                "axes": list(r.cfg["axes"]),
                "span": {a: c["span_deg"] for a, c in r.cfg["axes"].items()},
                "phases": [{"name": n, "how": h} for n, _d, h in _td.PHASES],
            }))

        if path == "/cmd":
            if "start" in q:
                from urllib.parse import unquote
                r.start_phase(unquote(q["start"]))
            elif "stop" in q:
                r.end_phase()
            elif "reset" in q:
                r.reset()
            return self._send("{}")

        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(r.snapshot())
                    self.wfile.write(("data: %s\n\n" % payload).encode())
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return

        self.send_response(404)
        self.send_header("Content-Length", "0")
        self.end_headers()


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=DEFAULT_CONFIG)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8410)
    p.add_argument("--demo", action="store_true",
                   help="donnees fabriquees, sans materiel")
    args = p.parse_args()

    if args.demo and not os.path.exists(args.config):
        cfg = {"camera": "DEMO-CAM", "axes": {
            "focus": {"device": "DEMO-FOC", "axis": [1.0, 0.0, 0.0],
                      "ref": [1.0, 0.0, 0.0, 0.0], "lo": 0.0,
                      "hi": math.radians(300.0), "span_deg": 300.0},
            "zoom": {"device": "DEMO-ZOO", "axis": [1.0, 0.0, 0.0],
                     "ref": [1.0, 0.0, 0.0, 0.0], "lo": 0.0,
                     "hi": math.radians(180.0), "span_deg": 180.0}}}
    else:
        if not os.path.exists(args.config):
            sys.exit("axes.json introuvable (%s). Lance calib-axis.py d'abord."
                     % args.config)
        cfg = lensaxis.load(args.config)

    runner = Runner(cfg, demo=args.demo)
    Handler.runner = runner
    threading.Thread(target=runner.loop, daemon=True).start()

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    srv.daemon_threads = True
    print("Instrument sur http://%s:%d%s"
          % (args.host, args.port, "   [DEMO]" if args.demo else ""))
    print("Depuis le Mac :  ssh -L %d:localhost:%d unreal"
          % (args.port, args.port))
    print("Ctrl-C pour arreter.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        runner.stop.set()
        print("\nArret.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
