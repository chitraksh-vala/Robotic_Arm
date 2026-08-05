"""
SO-ARM101 Dual-Arm IK Server
=============================
Two SO-ARM101s in a shared MuJoCo scene, side by side (parallel, facing +X).
Left arm base at y=+0.25 m, Right arm base at y=-0.25 m.

Usage:
  python3 dual_ik_server.py            # headless
  python3 dual_ik_server.py --viewer   # opens MuJoCo viewer

HTTP Endpoints (port 7842):
  GET  /state                → full state for both arms + blocks
  GET  /render               → base64 PNG
  GET  /viewer_status

  POST /ik        { arm: "L"|"R", x, y, z, wrist_roll?, smooth?, duration? }
  POST /move      { arm: "L"|"R", joints: [6 floats] }
  POST /gripper   { arm: "L"|"R", value: 0..1 }
  POST /home      { arm: "L"|"R"|"both" }
  POST /pipeline  { steps: [ { arm, type, ... }, ... ] }   ← dual pipeline
  POST /reset_blocks   → put blocks back to start positions
  POST /setcam    { az, el, dist }
  POST /open_viewer
"""

import os, sys, json, time, math, threading, base64, io, argparse, urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
import numpy as np

os.environ.setdefault("MUJOCO_GL", "wgl")
import mujoco
import mujoco.viewer
from PIL import Image

SCENE    = os.path.join(os.path.dirname(__file__), "scene_dual.xml")
PORT     = 7842

# ── Per-arm config ─────────────────────────────────────────────────────────
ARMS = {
    "L": {
        "joints":   ["L_shoulder_pan","L_shoulder_lift","L_elbow_flex","L_wrist_flex","L_wrist_roll"],
        "acts":     ["L_shoulder_pan","L_shoulder_lift","L_elbow_flex","L_wrist_flex","L_wrist_roll","L_gripper"],
        "ee_site":  "L_gripperframe",
        "tgt":      ["L_tgt_x","L_tgt_y","L_tgt_z"],
    },
    "R": {
        "joints":   ["R_shoulder_pan","R_shoulder_lift","R_elbow_flex","R_wrist_flex","R_wrist_roll"],
        "acts":     ["R_shoulder_pan","R_shoulder_lift","R_elbow_flex","R_wrist_flex","R_wrist_roll","R_gripper"],
        "ee_site":  "R_gripperframe",
        "tgt":      ["R_tgt_x","R_tgt_y","R_tgt_z"],
    },
}

# Block reset positions (matches scene XML)
BLOCK_RESETS = {
    "block_red":   [0.22,  0.06, 0.03],
    "block_green": [0.22,  0.00, 0.03],
    "block_blue":  [0.22, -0.06, 0.03],
}


# ══════════════════════════════════════════════════════════════════════════════
#  DLS IK — scratch-data solver, never touches live MjData
# ══════════════════════════════════════════════════════════════════════════════
class DLSIK:
    def __init__(self, m, ee_site, arm_joints, lam=0.1, alpha=0.5, max_iter=100, tol=1e-3):
        self.m = m
        self.d = mujoco.MjData(m)   # private scratch
        mujoco.mj_resetData(m, self.d)
        self.lam, self.alpha, self.max_iter, self.tol = lam, alpha, max_iter, tol
        self.ee_id   = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, ee_site)
        self.jnt_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in arm_joints]
        self.dof_ids = [m.joint(ji).dofadr[0] for ji in self.jnt_ids]

    def _get_q(self):
        return np.array([self.d.qpos[self.m.joint(ji).qposadr[0]] for ji in self.jnt_ids])

    def _set_q(self, q):
        for k, ji in enumerate(self.jnt_ids):
            self.d.qpos[self.m.joint(ji).qposadr[0]] = q[k]

    def _clip(self, q):
        lo = np.array([self.m.joint(ji).range[0] for ji in self.jnt_ids])
        hi = np.array([self.m.joint(ji).range[1] for ji in self.jnt_ids])
        return np.clip(q, lo, hi)

    def _jacobian(self):
        Jp = np.zeros((3, self.m.nv)); Jr = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, self.d, Jp, Jr, self.ee_id)
        return Jp[:, self.dof_ids]

    def solve(self, target, q0=None):
        q = self._clip(np.array(q0, dtype=float) if q0 is not None else self._get_q().copy())
        target = np.array(target, dtype=float)
        
        for it in range(self.max_iter):
            self._set_q(q)
            mujoco.mj_forward(self.m, self.d)  # Full forward dynamics, not just kinematics
            
            err = target - self.d.site_xpos[self.ee_id]
            err_norm = float(np.linalg.norm(err))
            
            if err_norm < self.tol:
                break
            
            J = self._jacobian()
            
            # Damped Least Squares: dq = J^T (J J^T + λ^2 I)^-1 e
            JJt = J @ J.T
            damping = self.lam ** 2
            
            # Adaptive damping based on conditioning
            condition = np.linalg.cond(JJt)
            if condition > 1e6:
                damping *= 10
            
            try:
                dq = J.T @ np.linalg.solve(JJt + damping * np.eye(3), err)
            except np.linalg.LinAlgError:
                # Fallback to pseudoinverse if singular
                dq = np.linalg.pinv(J) @ err
            
            # Adaptive step size
            step = self.alpha
            if it < 5:
                step = min(self.alpha, err_norm * 2.0)  # Larger steps early on
            
            q = self._clip(q + step * dq)
        
        # Final forward pass
        self._set_q(q)
        mujoco.mj_forward(self.m, self.d)
        final_err = float(np.linalg.norm(target - self.d.site_xpos[self.ee_id]))
        
        return q.tolist(), final_err, it + 1


# ══════════════════════════════════════════════════════════════════════════════
#  Simulation state — one physics world, two arm controllers
# ══════════════════════════════════════════════════════════════════════════════
class DualSimState:
    def __init__(self, use_viewer=False):
        self.lock = threading.Lock()
        self.m = mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_resetData(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)

        # Resolve IDs for both arms
        self.act_ids, self.jnt_ids, self.ee_ids, self.tgt_ids, self.ik = {}, {}, {}, {}, {}
        for arm, cfg in ARMS.items():
            self.act_ids[arm] = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                                 for n in cfg["acts"]}
            self.jnt_ids[arm] = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)
                                 for n in cfg["joints"] + [f"{arm}_gripper"]}
            self.ee_ids[arm]  = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, cfg["ee_site"])
            self.tgt_ids[arm] = [
                self.m.joint(mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, t)).qposadr[0]
                for t in cfg["tgt"]
            ]
            self.ik[arm] = DLSIK(self.m, cfg["ee_site"], cfg["joints"])

        # Initialize controls to match initial joint positions (CRITICAL for position actuators!)
        for arm, cfg in ARMS.items():
            for name in cfg["joints"]:
                jnt_id = self.jnt_ids[arm][name]
                qpos_val = self.d.qpos[self.m.joint(jnt_id).qposadr[0]]
                act_id = self.act_ids[arm][name]
                self.d.ctrl[act_id] = qpos_val
            # Initialize gripper
            grip_jnt_id = self.jnt_ids[arm][f"{arm}_gripper"]
            grip_qpos = self.d.qpos[self.m.joint(grip_jnt_id).qposadr[0]]
            grip_act_id = self.act_ids[arm][f"{arm}_gripper"]
            self.d.ctrl[grip_act_id] = grip_qpos

        # Block body IDs for state reporting
        self.block_ids = {
            name: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in BLOCK_RESETS
        }

        # Per-arm smooth interpolation state (protected by lock)
        self._interp = {
            arm: dict(start=np.zeros(5), target=np.zeros(5),
                      t0=0.0, dur=0.05, active=False)
            for arm in ARMS
        }

        # Renderer
        self.renderer = mujoco.Renderer(self.m, height=720, width=1280)
        self.cam_az, self.cam_el, self.cam_dist = 180.0, -22.0, 1.6

        # Pipeline
        self.pipeline_running = False
        self.pipeline_log     = []

        # IK stats per arm
        self.ik_stats = {arm: {"err_mm": 0.0, "iters": 0} for arm in ARMS}

        # Viewer
        self._viewer_handle = None
        self._use_viewer    = use_viewer
        self._viewer_open   = False
        self._run = True

        if use_viewer:
            self._launch_viewer()

        self._pth = threading.Thread(target=self._physics_loop, daemon=True)
        self._pth.start()
        if self._use_viewer and self._viewer_open:
            self._vth = threading.Thread(target=self._viewer_sync_loop, daemon=True)
            self._vth.start()

    # ── viewer ───────────────────────────────────────────────────────────────
    def _launch_viewer(self):
        try:
            self._viewer_handle = mujoco.viewer.launch_passive(
                self.m, self.d, show_left_ui=True, show_right_ui=True)
            self._viewer_open = True
            print("✓ MuJoCo viewer opened")
            if not hasattr(self, '_vth') or not self._vth.is_alive():
                self._vth = threading.Thread(target=self._viewer_sync_loop, daemon=True)
                self._vth.start()
        except Exception as ex:
            print(f"⚠  Viewer failed: {ex}")
            self._viewer_open = False

    def viewer_is_open(self):
        if self._viewer_handle is None: return False
        try: return self._viewer_handle.is_running()
        except: return False

    def _viewer_sync_loop(self):
        dt = 1.0 / 60.0
        while self._run:
            if not self.viewer_is_open(): break
            try:
                with self.lock:
                    self._viewer_handle.sync()
            except: break
            time.sleep(dt)

    # ── physics loop ─────────────────────────────────────────────────────────
    def _physics_loop(self):
        phys_dt = self.m.opt.timestep
        print(f"✓ Physics loop started, timestep={phys_dt:.4f}s")
        while self._run:
            with self.lock:
                t = self.d.time
                for arm, iv in self._interp.items():
                    if iv["active"]:
                        elapsed = t - iv["t0"]
                        u = min(elapsed / iv["dur"], 1.0)
                        ts = u * u * (3.0 - 2.0 * u)   # smoothstep
                        ctrl = iv["start"] + ts * (iv["target"] - iv["start"])
                        joints = ARMS[arm]["joints"]
                        for i, name in enumerate(joints):
                            self.d.ctrl[self.act_ids[arm][name]] = float(ctrl[i])
                        if u >= 1.0:
                            iv["active"] = False
                            for i, name in enumerate(joints):
                                self.d.ctrl[self.act_ids[arm][name]] = float(iv["target"][i])
                            print(f"  [{arm}] Interpolation complete at t={t:.2f}s")
                mujoco.mj_step(self.m, self.d)
            time.sleep(phys_dt * 0.5)

    # ── helpers ───────────────────────────────────────────────────────────────
    def _start_interp(self, arm, target, duration):
        """Must be called with lock held."""
        current = np.array([float(self.d.ctrl[self.act_ids[arm][n]])
                            for n in ARMS[arm]["joints"]])
        iv = self._interp[arm]
        iv["start"]  = current
        iv["target"] = np.array(target, dtype=float)
        iv["t0"]     = self.d.time
        iv["dur"]    = max(duration, 0.05)
        iv["active"] = True

    # ── IK ───────────────────────────────────────────────────────────────────
    def solve_ik(self, arm, x, y, z, wrist_roll=None, smooth=True, duration=2.0):
        with self.lock:
            # Use CURRENT JOINT POSITIONS as q0, not just ctrl values
            # This is more accurate especially on first IK call
            q0 = [float(self.d.qpos[self.m.joint(self.jnt_ids[arm][n]).qposadr[0]]) 
                  for n in ARMS[arm]["joints"]]
            self.d.qpos[self.tgt_ids[arm][0]] = x
            self.d.qpos[self.tgt_ids[arm][1]] = y
            self.d.qpos[self.tgt_ids[arm][2]] = z

        q, err, iters = self.ik[arm].solve([x, y, z], q0=q0)
        
        # Override wrist_roll if specified (joint index 4)
        if wrist_roll is not None:
            q[4] = float(wrist_roll)
        
        self.ik_stats[arm] = {"err_mm": round(err * 1000, 2), "iters": iters}
        print(f"  IK [{arm}] → [{x:.3f},{y:.3f},{z:.3f}]  err={err*1000:.1f}mm  iters={iters}")
        print(f"     q0={[f'{v:.3f}' for v in q0]}")
        print(f"     q_target={[f'{v:.3f}' for v in q]}")
        if wrist_roll is not None:
            print(f"     wrist_roll={wrist_roll:.3f} rad ({np.degrees(wrist_roll):.1f}°)")

        with self.lock:
            if smooth:
                self._start_interp(arm, q, duration)
                print(f"     Started interpolation for {duration}s, active={self._interp[arm]['active']}")
            else:
                for i, name in enumerate(ARMS[arm]["joints"]):
                    self.d.ctrl[self.act_ids[arm][name]] = q[i]
        return q, err, iters

    def set_joints(self, arm, joints):
        with self.lock:
            acts = ARMS[arm]["acts"]
            for i, name in enumerate(acts[:len(joints)]):
                self.d.ctrl[self.act_ids[arm][name]] = joints[i]

    def set_gripper(self, arm, value):
        ctrl = 1.7453 - value * (1.7453 + 0.17453)
        with self.lock:
            self.d.ctrl[self.act_ids[arm][f"{arm}_gripper"]] = ctrl

    def home(self, arm="both"):
        targets = ["L", "R"] if arm == "both" else [arm]
        with self.lock:
            for a in targets:
                for name in ARMS[a]["acts"]:
                    self.d.ctrl[self.act_ids[a][name]] = 0.0
                self._start_interp(a, np.zeros(5), 1.2)

    def reset_blocks(self):
        """Teleport blocks back to starting positions."""
        with self.lock:
            for name, pos in BLOCK_RESETS.items():
                bid = self.block_ids[name]
                # Find the freejoint qpos address for this body
                # freejoint = first joint of the body; qpos = [x,y,z, qw,qx,qy,qz]
                body = self.m.body(bid)
                jid  = body.jntadr[0]
                qadr = self.m.joint(jid).qposadr[0]
                self.d.qpos[qadr:qadr+3] = pos
                self.d.qpos[qadr+3] = 1.0   # quaternion w
                self.d.qpos[qadr+4:qadr+7] = 0.0
                self.d.qvel[self.m.joint(jid).dofadr[0]:
                            self.m.joint(jid).dofadr[0]+6] = 0.0
            mujoco.mj_forward(self.m, self.d)

    # ── pipeline ──────────────────────────────────────────────────────────────
    def run_pipeline(self, steps):
        """
        Dual-arm pipeline.  Each step has an 'arm' field: "L", "R", or "both".
        Steps without 'arm' default to "L" for backward compat.
        Parallel steps (same timestamp / marked parallel=True) run concurrently.
        """
        def _exec():
            self.pipeline_running = True
            self.pipeline_log     = []
            i = 0
            while i < len(steps):
                step = steps[i]
                parallel_group = [step]
                # Collect consecutive steps marked parallel=True
                while i + 1 < len(steps) and steps[i + 1].get("parallel", False):
                    i += 1
                    parallel_group.append(steps[i])

                if len(parallel_group) == 1:
                    _run_step(step)
                else:
                    # Run group concurrently
                    threads = [threading.Thread(target=_run_step, args=(s,), daemon=True)
                               for s in parallel_group]
                    for t in threads: t.start()
                    for t in threads: t.join()
                i += 1

            self.pipeline_running = False
            self.pipeline_log.append("✅ Pipeline complete")

        def _run_step(step):
            arm      = step.get("arm", "L")
            stype    = step.get("type", "move_ik")
            label    = step.get("label", stype)
            self.pipeline_log.append(f"▶ [{arm}] {label}")

            if stype == "move_ik":
                x, y, z     = float(step["x"]), float(step["y"]), float(step["z"])
                duration    = float(step.get("duration", 2.0))
                settle_time = float(step.get("settle_time", 0.5))
                wrist_roll  = step.get("wrist_roll", None)
                if wrist_roll is not None:
                    wrist_roll = float(wrist_roll)
                if arm == "both":
                    # Solve IK for both arms simultaneously
                    threads = []
                    for a in ["L", "R"]:
                        t = threading.Thread(
                            target=lambda a=a: self.solve_ik(a, x, y, z, wrist_roll=wrist_roll, smooth=True, duration=duration),
                            daemon=True)
                        threads.append(t)
                    for t in threads: t.start()
                    for t in threads: t.join()
                else:
                    self.solve_ik(arm, x, y, z, wrist_roll=wrist_roll, smooth=True, duration=duration)
                time.sleep(duration + settle_time)
                stats = self.ik_stats.get(arm if arm != "both" else "L", {})
                self.pipeline_log.append(
                    f"  ✓ err={stats.get('err_mm','?')}mm ({duration:.1f}s + {settle_time:.1f}s)")

            elif stype == "gripper":
                value = float(step.get("value", 0.0))
                if arm == "both":
                    for a in ["L", "R"]: self.set_gripper(a, value)
                else:
                    self.set_gripper(arm, value)
                time.sleep(float(step.get("settle_time", 0.8)))
                self.pipeline_log.append(
                    f"  ✓ Gripper {'OPEN' if value < 0.5 else 'CLOSE'}")

            elif stype == "home":
                self.home(arm)
                time.sleep(1.5)
                self.pipeline_log.append("  ✓ Home")

            elif stype == "wait":
                dur = float(step.get("duration", 1.0))
                time.sleep(dur)
                self.pipeline_log.append(f"  ✓ Wait {dur:.1f}s")

            elif stype == "reset_blocks":
                self.reset_blocks()
                time.sleep(0.5)
                self.pipeline_log.append("  ✓ Blocks reset")

        threading.Thread(target=_exec, daemon=True).start()

    # ── state / render ────────────────────────────────────────────────────────
    def get_state(self):
        with self.lock:
            mujoco.mj_forward(self.m, self.d)
            state = {"arms": {}, "blocks": {}, "pipeline_running": self.pipeline_running,
                     "pipeline_log": self.pipeline_log[-40:], "viewer_open": self.viewer_is_open()}
            for arm, cfg in ARMS.items():
                ee  = self.d.site_xpos[self.ee_ids[arm]].copy()
                jq  = {n: float(self.d.qpos[self.m.joint(self.jnt_ids[arm][n]).qposadr[0]])
                       for n in cfg["joints"] + [f"{arm}_gripper"]}
                ctrl = {n: float(self.d.ctrl[self.act_ids[arm][n]]) for n in cfg["acts"]}
                state["arms"][arm] = {
                    "ee": ee.tolist(), "joints": jq, "ctrl": ctrl,
                    "ik_err_mm": self.ik_stats[arm]["err_mm"],
                    "ik_iters":  self.ik_stats[arm]["iters"],
                }
            for name, bid in self.block_ids.items():
                pos = self.d.xpos[bid].copy()
                state["blocks"][name] = pos.tolist()
        return state

    def render_png(self):
        with self.lock:
            qpos_c = self.d.qpos.copy(); qvel_c = self.d.qvel.copy(); ctrl_c = self.d.ctrl.copy()
        d_r = mujoco.MjData(self.m)
        d_r.qpos[:] = qpos_c; d_r.qvel[:] = qvel_c; d_r.ctrl[:] = ctrl_c
        mujoco.mj_forward(self.m, d_r)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.azimuth, cam.elevation, cam.distance = self.cam_az, self.cam_el, self.cam_dist
        cam.lookat[:] = [0.22, 0.0, 0.1]
        self.renderer.update_scene(d_r, camera=cam)
        px = self.renderer.render()
        buf = io.BytesIO()
        Image.fromarray(px).save(buf, format="PNG")
        return base64.b64encode(buf.getvalue()).decode()


# ══════════════════════════════════════════════════════════════════════════════
#  HTTP Server
# ══════════════════════════════════════════════════════════════════════════════
SIM: DualSimState = None

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass
    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
    def send_json(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self.send_header("Content-Length", str(len(body)))
        self._cors(); self.end_headers(); self.wfile.write(body)
    def read_json(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n)) if n else {}
    def do_OPTIONS(self):
        self.send_response(200); self._cors(); self.end_headers()

    def do_GET(self):
        if self.path == "/state":        self.send_json(SIM.get_state())
        elif self.path == "/render":     self.send_json({"png": SIM.render_png()})
        elif self.path == "/viewer_status": self.send_json({"open": SIM.viewer_is_open()})
        else: self.send_json({"error": "not found"}, 404)

    def do_POST(self):
        body = self.read_json()
        p = self.path

        if p == "/ik":
            arm = body.get("arm", "L")
            if arm not in ARMS: return self.send_json({"error": "arm must be L or R"}, 400)
            wrist_roll = body.get("wrist_roll", None)
            if wrist_roll is not None:
                wrist_roll = float(wrist_roll)
            q, err, iters = SIM.solve_ik(
                arm, float(body["x"]), float(body["y"]), float(body["z"]),
                wrist_roll=wrist_roll,
                smooth=body.get("smooth", True), duration=float(body.get("duration", 2.0)))
            self.send_json({"arm": arm, "joints": dict(zip(ARMS[arm]["joints"], q)),
                            "err_mm": round(err*1000,2), "iters": iters,
                            "state": SIM.get_state(), "png": SIM.render_png()})

        elif p == "/move":
            arm = body.get("arm", "L")
            joints = body.get("joints", [])
            if isinstance(joints, dict):
                joints = [joints.get(n, 0.0) for n in ARMS[arm]["acts"]]
            SIM.set_joints(arm, joints)
            self.send_json({"ok": True})

        elif p == "/gripper":
            arm = body.get("arm", "L")
            SIM.set_gripper(arm, float(body.get("value", 0)))
            self.send_json({"ok": True})

        elif p == "/home":
            SIM.home(body.get("arm", "both"))
            self.send_json({"ok": True})

        elif p == "/pipeline":
            SIM.run_pipeline(body["steps"])
            self.send_json({"ok": True, "msg": "Pipeline started"})

        elif p == "/reset_blocks":
            SIM.reset_blocks()
            self.send_json({"ok": True})

        elif p == "/setcam":
            SIM.cam_az   = body.get("az",   SIM.cam_az)
            SIM.cam_el   = body.get("el",   SIM.cam_el)
            SIM.cam_dist = body.get("dist", SIM.cam_dist)
            self.send_json({"ok": True})

        elif p == "/open_viewer":
            if not SIM.viewer_is_open():
                SIM._launch_viewer()
            self.send_json({"ok": True, "open": SIM.viewer_is_open()})

        else:
            self.send_json({"error": "not found"}, 404)


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════
def main():
    global SIM
    ap = argparse.ArgumentParser(description="SO-ARM101 Dual-Arm IK Server")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--port",   type=int, default=PORT)
    ap.add_argument("--scene",  default=SCENE)
    args = ap.parse_args()

    print("=" * 60)
    print("  SO-ARM101  DUAL-ARM IK Server")
    print("  Left arm: y=+0.25   Right arm: y=-0.25")
    print(f"  Mode: {'LIVE VIEWER' if args.viewer else 'HEADLESS'}")
    print("=" * 60)

    SIM = DualSimState(use_viewer=args.viewer)

    for arm in ["L", "R"]:
        ee = SIM.d.site_xpos[SIM.ee_ids[arm]]
        print(f"  [{arm}] EE home: [{ee[0]:.4f}, {ee[1]:.4f}, {ee[2]:.4f}]")
    print(f"  HTTP port: {args.port}")
    print()

    srv = HTTPServer(("127.0.0.1", args.port), Handler)
    print(f"✓ Server ready → http://127.0.0.1:{args.port}")
    print("  Open dual_dashboard.html in your browser.")
    print()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        SIM._run = False
        if SIM._viewer_handle:
            try: SIM._viewer_handle.close()
            except: pass

if __name__ == "__main__":
    main()
