#!/usr/bin/env python3
"""Browser control panel for the arm.

    python control.py                 # auto-detect the port
    python control.py --port COM7
    python control.py --simulate      # no hardware needed
    python control.py --list

Stdlib HTTP server + armlink. Close any serial monitor first: one program per
COM port.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from armlink import connect, list_serial_ports          # noqa: E402
from armlink.arm import Arm                             # noqa: E402

HERE = Path(__file__).parent
POSE_FILE = HERE / "poses.json"
TWIN_PAGE = HERE / "web" / "twin.html"
DESCRIPTION = HERE.parent / "description"
# Only these are served from description/ - no path from the URL ever reaches
# the filesystem unchecked.
STATIC_OK = re.compile(r"^/description/(arm\.json|meshes/[a-z0-9_]+\.stl)$")


class Model:
    """The kinematic model, reloaded whenever arm.json changes on disk, so a
    sign flip or a measured TCP takes effect on the next request."""

    def __init__(self, path: Path):
        self.path, self.mtime, self.model = path, None, None

    def get(self):
        from armlink.kinematics import ArmModel     # numpy only needed for the twin
        m = self.path.stat().st_mtime
        if self.model is None or m != self.mtime:
            self.model, self.mtime = ArmModel(self.path), m
        return self.model


class Poses:
    """Named joint poses, saved next to this script.

    Capturing a pose you reached by jogging and replaying it later is the
    smallest useful step towards trajectories, and it costs almost nothing.
    """

    def __init__(self, path: Path):
        self.path = path
        # {pose_name: {joint_name: degrees}}. Older files stored a positional
        # list, which silently broke when the joint set changed - a pose saved
        # against six joints was replayed against three. Migrate on load.
        self.data: dict[str, dict] = {}
        if path.exists():
            try:
                self.data = json.loads(path.read_text())
            except Exception:
                self.data = {}
            for k, v in list(self.data.items()):
                if isinstance(v, list):
                    self.data[k] = {f"J{i+1}": float(x) for i, x in enumerate(v)}

    def save(self, name: str, mapping: dict):
        self.data[name] = {k: round(v, 4) for k, v in mapping.items()}
        self._flush()

    def delete(self, name: str):
        self.data.pop(name, None)
        self._flush()

    def _flush(self):
        self.path.write_text(json.dumps(self.data, indent=2))


class Handler(BaseHTTPRequestHandler):
    arm: Arm = None
    poses: Poses = None
    model = Model(DESCRIPTION / "arm.json")

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith("/api/state"):
            st = self.arm.state()
            self._send(200, json.dumps({
                "enabled": st.enabled, "moving": st.moving, "estop": st.estopped,
                "joints": [{
                    "name": j.name, "pos": j.position, "moving": j.moving,
                    "attached": j.attached, "present": j.present, "kind": j.kind,
                    "driver": j.driver_online, "min": j.min_deg, "max": j.max_deg,
                    "grip": j.extra.get("grip"),
                } for j in st.joints],
                "poses": self.poses.data,
                "events": self.arm.link.take_events(),
            }))
        elif self.path in ("/", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/twin":
            self._send(200, TWIN_PAGE.read_bytes(), "text/html; charset=utf-8")
        elif STATIC_OK.match(self.path):
            f = DESCRIPTION / self.path[len("/description/"):]
            if f.is_file():
                ctype = "application/json" if f.suffix == ".json" else "model/stl"
                self._send(200, f.read_bytes(), ctype)
            else:
                self._send(404, "{}")
        else:
            self._send(404, "{}")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            self._send(400, json.dumps({"ok": False, "text": "bad json"}))
            return

        action = req.get("action", "")
        try:
            if action == "cmd":
                r = self.arm.cmd(str(req["line"]), timeout=float(req.get("timeout", 10)))
                self._send(200, json.dumps({
                    "ok": r.ok, "text": str(r), "lines": r.raw}))
            elif action == "pose_save":
                self.poses.save(str(req["name"]), self.arm.state().by_name())
                self._send(200, json.dumps({"ok": True, "text": "saved"}))
            elif action == "pose_delete":
                self.poses.delete(str(req["name"]))
                self._send(200, json.dumps({"ok": True, "text": "deleted"}))
            elif action == "pose_go":
                target = self.poses.data.get(str(req["name"]))
                if not target:
                    self._send(200, json.dumps({"ok": False, "text": "no such pose"}))
                    return
                r = self.arm.goto_named(target, feed=float(req.get("feed", 0.4)))
                self._send(200, json.dumps({"ok": r.ok, "text": str(r), "lines": r.raw}))
            elif action == "ik":
                self._ik(req)
            elif action == "go_model":
                m = self.model.get()
                fw = m.to_firmware([float(v) for v in req["q"]])
                r = self.arm.goto_named(fw, feed=float(req.get("feed", 0.3)))
                self._send(200, json.dumps({"ok": r.ok, "text": str(r), "fw": fw}))
            elif action == "grip":
                verb = str(req.get("verb", ""))
                fn = {"close": self.arm.grip_close, "open": self.arm.grip_open,
                      "stop": self.arm.grip_stop}.get(verb)
                if fn is None:
                    self._send(400, json.dumps({"ok": False, "text": "verb: close|open|stop"}))
                    return
                r = fn()
                self._send(200, json.dumps({"ok": r.ok, "text": str(r)}))
            else:
                self._send(400, json.dumps({"ok": False, "text": "bad action"}))
        except Exception as e:
            self._send(200, json.dumps({"ok": False, "text": f"host error: {e}"}))

    def _ik(self, req: dict):
        """Solve for the twin's target. Pure maths - nothing moves until the
        page sends go_model with the answer."""
        from armlink.kinematics import quat_to_matrix
        m = self.model.get()
        seed = req.get("seed") or [0.0] * m.n
        quat = req.get("quat")
        sol = m.ik([float(v) for v in req["pos"]],
                   quat_to_matrix(quat) if quat else None,
                   seed=[float(v) for v in seed],
                   restarts=2 if quat is None else 6)
        self._send(200, json.dumps({
            "ok": sol.ok, "q": sol.q, "pos_err_mm": sol.pos_err_mm,
            "rot_err_deg": sol.rot_err_deg, "reason": sol.reason,
            "fw": m.to_firmware(sol.q)}))


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Arm Control</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#101319;--panel:#191d26;--line:#2a2f3c;--fg:#e7eaf0;--dim:#8c94a6;
      --accent:#5aa9e6;--good:#4ade80;--warn:#fbbf24;--bad:#f87171}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.5 ui-sans-serif,system-ui,Segoe UI,Roboto,sans-serif}
header{display:flex;gap:12px;align-items:center;padding:10px 16px;background:var(--panel);
       border-bottom:1px solid var(--line);position:sticky;top:0;z-index:5;flex-wrap:wrap}
h1{font-size:15px;margin:0;font-weight:600}
.wrap{display:grid;grid-template-columns:minmax(340px,1.2fr) minmax(300px,1fr);
      gap:14px;padding:14px;align-items:start}
@media(max-width:900px){.wrap{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px}
.card h2{font-size:11px;text-transform:uppercase;letter-spacing:.09em;color:var(--dim);
         margin:0 0 12px}
button{background:#242935;color:var(--fg);border:1px solid var(--line);border-radius:7px;
       padding:7px 11px;font-size:13px;cursor:pointer}
button:hover{background:#2d3340}
button.p{background:var(--accent);border-color:var(--accent);color:#06121d;font-weight:600}
button.stop{background:var(--bad);border-color:var(--bad);color:#1a0505;font-weight:700}
input[type=text],input[type=number]{background:#0c0f14;color:var(--fg);border:1px solid var(--line);
  border-radius:6px;padding:6px 8px;font-size:13px;width:100%}
input[type=range]{width:100%;accent-color:var(--accent)}
.jrow.absent{opacity:.42}
.jrow.absent .jval{font-size:11px;color:var(--dim)}
.jrow{display:grid;grid-template-columns:42px 1fr 74px auto auto;gap:8px;align-items:center;
      margin-bottom:9px}
.jname{color:var(--dim);font-weight:600}
.jval{text-align:right;font-variant-numeric:tabular-nums}
.pill{padding:2px 9px;border-radius:99px;font-size:11px;font-weight:600;border:1px solid var(--line)}
.ok{color:var(--good);border-color:#2b5138}.off{color:var(--dim)}
.bad{color:var(--bad);border-color:#5a2b2b}.warnp{color:var(--warn);border-color:#5a4a1e}
.row{display:flex;gap:8px;align-items:center;margin-bottom:10px;flex-wrap:wrap}
.row>label{color:var(--dim);min-width:52px;font-size:12px}
#log{height:210px;overflow:auto;background:#0c0f14;border:1px solid var(--line);border-radius:6px;
     padding:8px;font:12px/1.45 ui-monospace,Consolas,monospace;white-space:pre-wrap;color:var(--dim)}
#log .c{color:var(--accent)}#log .e{color:var(--bad)}#log .v{color:var(--warn)}
.poses{display:flex;flex-wrap:wrap;gap:6px}
.poses button{font-size:12px;padding:5px 9px}
</style></head><body>

<header>
  <h1>Arm Control</h1>
  <span id="pEn" class="pill off">motors</span>
  <span id="pMv" class="pill off">idle</span>
  <span id="pEs" class="pill off">estop</span>
  <span style="flex:1"></span>
  <a href="/twin" style="color:var(--accent);text-decoration:none;font-size:13px;margin-right:6px">3D twin &rarr;</a>
  <button id="bEn">Enable</button>
  <button id="bStop">Stop</button>
  <button id="bEstop" class="stop">E-STOP</button>
</header>

<div class="wrap">
  <div class="card">
    <h2>Joints</h2>
    <div id="joints"></div>
    <div class="row" style="margin-top:14px">
      <label>feed</label>
      <input type="range" id="feed" min="0.05" max="1" step="0.05" value="0.4" style="flex:1">
      <span id="feedv" style="min-width:34px;text-align:right">0.40</span>
    </div>
    <div class="row">
      <label>step</label>
      <input type="number" id="step" value="5" step="1" style="width:80px"><span style="color:var(--dim)">deg per jog click</span>
    </div>
    <div class="row">
      <button id="bZero">Teach all = 0</button>
      <button id="bHome">Go to 0,0,0</button>
    </div>
    <div class="row">
      <label>gripper</label>
      <button data-grip="close">Close</button>
      <button data-grip="open">Open</button>
      <button data-grip="stop">Stop</button>
      <span id="gstate" style="color:var(--dim);font-size:12px"></span>
    </div>
  </div>

  <div class="card">
    <h2>Poses</h2>
    <div class="row">
      <input type="text" id="poseName" placeholder="name this pose">
      <button class="p" id="bCapture">Capture</button>
    </div>
    <div class="poses" id="poseList"></div>
    <div class="hint" style="color:var(--dim);font-size:12px;margin-top:10px">
      Click a pose to move there. Shift-click to delete.
    </div>
  </div>

  <div class="card" style="grid-column:1/-1">
    <h2>Console</h2>
    <div class="row">
      <input type="text" id="raw" placeholder="any firmware command, e.g.  diag units 0">
      <button class="p" id="bSend">Send</button>
    </div>
    <div id="log"></div>
  </div>
</div>

<script>
const $=id=>document.getElementById(id);
let S={joints:[],poses:{}},busy=false,built=false;

function log(t,cls){const l=$("log"),s=document.createElement("span");
  if(cls)s.className=cls;s.textContent=t+"\n";l.appendChild(s);l.scrollTop=l.scrollHeight;}

async function post(body){
  try{const r=await fetch("/api",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify(body)});return await r.json();}
  catch(e){log("host error: "+e,"e");return{ok:false};}
}
async function cmd(line,timeout){
  log("> "+line,"c");
  busy=true; const j=await post({action:"cmd",line:line,timeout:timeout||10}); busy=false;
  (j.lines||[]).forEach(l=>log("  "+l, l.startsWith("err")?"e":null));
  if(!j.ok&&!(j.lines||[]).some(l=>l.startsWith("err")))log("  "+(j.text||"failed"),"e");
  return j;
}

function buildJoints(js){
  $("joints").innerHTML = js.map((j,i)=>{
    const off = !(j.present && j.attached);
    return `
    <div class="jrow${off?" absent":""}">
      <span class="jname">${j.name}</span>
      <input type="range" id="s${i}" min="${j.min}" max="${j.max}" step="0.5"
             value="${j.pos}" ${off?"disabled":""}>
      <span class="jval" id="v${i}">${off?(j.kind==="servo"?"servo — not built":"not attached")
                                        :j.pos.toFixed(2)+"&deg;"}</span>
      <button data-jog="${i},-1" ${off?"disabled":""}>-</button>
      <button data-jog="${i},1" ${off?"disabled":""}>+</button>
    </div>`;}).join("");
  js.forEach((j,i)=>{
    if(!(j.present&&j.attached))return;
    const s=$("s"+i);
    s.oninput=()=>$("v"+i).textContent=(+s.value).toFixed(2)+"°";
    s.onchange=()=>cmd(`move ${i} ${(+s.value).toFixed(3)} ${$("feed").value}`);
  });
  document.querySelectorAll("[data-jog]").forEach(b=>b.onclick=()=>{
    const [i,sgn]=b.dataset.jog.split(",").map(Number);
    cmd(`jog ${i} ${(sgn*(+$("step").value)).toFixed(3)} ${$("feed").value}`);
  });
  built=true;
}

function pill(el,on,txt,cls){el.textContent=txt;el.className="pill "+(on?(cls||"ok"):"off");}

function renderPoses(p){
  const names=Object.keys(p);
  $("poseList").innerHTML = names.length
    ? names.map(n=>`<button data-pose="${n}">${n}</button>`).join("")
    : '<span style="color:var(--dim);font-size:12px">none yet</span>';
  document.querySelectorAll("[data-pose]").forEach(b=>b.onclick=async ev=>{
    const n=b.dataset.pose;
    if(ev.shiftKey){await post({action:"pose_delete",name:n});log("deleted pose "+n,"v");return;}
    log("> pose "+n,"c");
    const j=await post({action:"pose_go",name:n,feed:+$("feed").value});
    if(!j.ok)log("  "+j.text,"e");
  });
}

async function poll(){
  if(!busy){
    try{
      const r=await fetch("/api/state");const st=await r.json();S=st;
      if(!built&&st.joints.length)buildJoints(st.joints);
      st.joints.forEach((j,i)=>{
        if(!(j.present&&j.attached))return;
        const s=$("s"+i);
        if(s&&document.activeElement!==s){s.value=j.pos;$("v"+i).textContent=j.pos.toFixed(2)+"°";}
      });
      const g=st.joints.find(j=>j.name==="GRIP");
      if(g)$("gstate").textContent=(g.grip||"")+" "+g.pos.toFixed(0)+"% wound";
      pill($("pEn"),st.enabled,st.enabled?"MOTORS ON":"motors off");
      pill($("pMv"),st.moving,st.moving?"MOVING":"idle",st.moving?"warnp":"");
      pill($("pEs"),st.estop,st.estop?"E-STOPPED":"estop clear",st.estop?"bad":"off");
      renderPoses(st.poses||{});
      (st.events||[]).forEach(e=>log("! "+e,"v"));
    }catch(e){}
  }
  setTimeout(poll,300);
}

$("feed").oninput=()=>$("feedv").textContent=(+$("feed").value).toFixed(2);
$("bEn").onclick=()=>cmd("enable "+(S.enabled?0:1));
$("bStop").onclick=()=>cmd("stop");
$("bEstop").onclick=()=>cmd(S.estop?"estop clear":"estop");
$("bZero").onclick=()=>cmd("teach all 0");
document.querySelectorAll("[data-grip]").forEach(b=>b.onclick=()=>cmd("grip "+b.dataset.grip));
$("bHome").onclick=()=>cmd(`goto 0 0 0 - - - ${$("feed").value}`);
$("bCapture").onclick=async()=>{
  const n=$("poseName").value.trim();if(!n)return;
  await post({action:"pose_save",name:n});$("poseName").value="";log("captured pose "+n,"v");
};
$("bSend").onclick=()=>{const v=$("raw").value.trim();if(v){cmd(v,60);$("raw").value="";}};
$("raw").onkeydown=e=>{if(e.key==="Enter")$("bSend").click();};
document.onkeydown=e=>{
  if(document.activeElement.tagName==="INPUT")return;
  if(e.key===" "){$("bEstop").click();e.preventDefault();}
};
poll();
</script></body></html>
"""


def serve_somewhere(preferred: int) -> ThreadingHTTPServer:
    """Windows reports an occupied wildcard bind as access-denied, not
    address-in-use, and something usually holds 8080. Walk candidates, then
    let the OS choose."""
    last = None
    for port in [preferred, 8000, 8123, 7070, 9090, 5151, 0]:
        try:
            return ThreadingHTTPServer(("127.0.0.1", port), Handler)
        except OSError as e:
            last = e
    raise SystemExit(f"could not bind any port: {last}")


def main():
    ap = argparse.ArgumentParser(description="browser control panel for the arm")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http", type=int, default=8080)
    ap.add_argument("--simulate", action="store_true", help="no hardware needed")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    if args.list:
        for dev, desc in list_serial_ports():
            print(f"{dev:10s} {desc}")
        return

    try:
        arm = connect(args.port, args.baud, simulate=args.simulate)
    except SystemExit:
        raise
    except Exception as e:
        print(f"\ncould not connect: {e}\n")
        print("ports this machine can see:")
        found = list_serial_ports()
        for dev, desc in found:
            print(f"  {dev:10s} {desc}")
        if not found:
            print("  (none - is the board plugged in?)")
        print("\n  * a serial monitor may still hold the port - close it")
        print("  * wrong port chosen automatically -> --port COMx")
        print("  * --simulate explores the interface with no hardware")
        raise SystemExit(1)

    print("connected." if not args.simulate else "SIMULATED arm - no serial port in use.")
    print("joints:", ", ".join(j.name for j in arm.meta))

    Handler.arm = arm
    Handler.poses = Poses(POSE_FILE)

    # /api and /api/state share one handler; route by method.
    Handler.do_POST = Handler.do_POST
    srv = serve_somewhere(args.http)
    url = f"http://localhost:{srv.server_address[1]}"
    if srv.server_address[1] != args.http:
        print(f"port {args.http} unavailable, using {srv.server_address[1]}")
    print(f"\ncontrol panel: {url}\nCtrl+C to stop\n")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nstopping")
        try:
            arm.stop()
        except Exception:
            pass
        arm.close()


if __name__ == "__main__":
    main()
