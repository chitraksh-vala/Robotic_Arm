"""
SO-ARM101 IK Simulation Server  (with live MuJoCo viewer support)
==================================================================
Usage:
  python3 ik_server.py           # headless (EGL render → dashboard PNG)
  python3 ik_server.py --viewer  # opens the MuJoCo interactive viewer window

The --viewer flag launches mujoco.viewer.launch_passive() which opens the
full interactive 3-D viewer. You can orbit, zoom, inspect contacts, etc.
The viewer window stays in sync with every IK command from the dashboard.

HTTP endpoints (port 7842):
  GET  /state         → EE pos, joint angles, IK stats, pipeline log
  GET  /render        → base64 PNG from offscreen renderer (always works)
  POST /ik            { x, y, z, wrist_angle? }
  POST /move          { joints: [6 floats] }
  POST /gripper       { value: 0..1 }
  POST /pipeline      { steps: [...] }
  POST /setcam        { az, el, dist }
  POST /home
  GET  /viewer_status → whether viewer window is open

Pipeline step format:
  move_ik steps now support optional wrist_angle parameter:
  { type: "move_ik", x, y, z, duration?, settle_time?, wrist_angle? }
  
  wrist_angle: Optional wrist_roll angle in radians (controls gripper rotation)
    π/2 (~1.57) = horizontal gripper fingers, parallel to floor (for pick-and-place)
    0 = default orientation (gripper fingers vertical)
    -π/2 (~-1.57) = horizontal gripper fingers, opposite rotation
    If omitted, wrist_roll angle is determined naturally by IK solver
"""

import os, sys, math, json, time, threading, base64, io, argparse, urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
import numpy as np

# ── GL backend: use EGL for offscreen; viewer manages its own context ────────
os.environ.setdefault("MUJOCO_GL", "wgl")
import mujoco
import mujoco.viewer
from PIL import Image

SCENE      = os.path.join(os.path.dirname(__file__), "scene_full.xml")
ARM_JOINTS = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ALL_ACTS   = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
EE_SITE    = "gripperframe"
PORT       = 7842

# ── Optional hardware bridge mirror ──────────────────────────────────────────
HW_BRIDGE_URL = "http://127.0.0.1:7843"

def mirror_hw(path, payload):
    """Mirror sim motion to hardware bridge if available."""
    try:
        req = urllib.request.Request(
            HW_BRIDGE_URL + path,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        urllib.request.urlopen(req, timeout=1)
    except Exception as e:
        print(f"  HW mirror failed for {path}: {e}")
# ══════════════════════════════════════════════════════════════════════════════
#  DLS IK Solver
# ══════════════════════════════════════════════════════════════════════════════
class DLSIK:
    def __init__(self, m, d, ee_site, arm_joints,
                 lam=0.015, alpha=0.4, max_iter=50, tol=5e-4):
        self.m, self.d = m, d
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

    def solve(self, target):
        q = self._get_q().copy()
        for it in range(self.max_iter):
            self._set_q(q)
            mujoco.mj_kinematics(self.m, self.d)
            err = np.array(target) - self.d.site_xpos[self.ee_id]
            en  = float(np.linalg.norm(err))
            if en < self.tol:
                break
            J   = self._jacobian()
            JJt = J @ J.T
            lam = self.lam if float(np.linalg.det(JJt)) > 1e-8 else self.lam * 20
            q   = self._clip(q + self.alpha * J.T @ np.linalg.solve(JJt + lam**2 * np.eye(3), err))
        self._set_q(q)
        mujoco.mj_kinematics(self.m, self.d)
        final_err = float(np.linalg.norm(np.array(target) - self.d.site_xpos[self.ee_id]))
        return q.tolist(), final_err, it + 1


# ══════════════════════════════════════════════════════════════════════════════
#  Simulation State  (physics loop + optional viewer)
# ══════════════════════════════════════════════════════════════════════════════
class SimState:
    def __init__(self, use_viewer: bool = False):
        self.lock = threading.Lock()
        self.m    = mujoco.MjModel.from_xml_path(SCENE)
        self.d    = mujoco.MjData(self.m)
        mujoco.mj_resetData(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)

        # Create scratch copy of MjData for IK solver (never touches live sim data)
        self.d_scratch = mujoco.MjData(self.m)
        self.ik      = DLSIK(self.m, self.d_scratch, EE_SITE, ARM_JOINTS)
        self.ee_id   = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
        self.act_ids = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ALL_ACTS}
        self.jnt_ids = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)
                        for n in ARM_JOINTS + ["gripper"]}
        self.tgt_ids = [
            self.m.joint(mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "tgt_x")).qposadr[0],
            self.m.joint(mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "tgt_y")).qposadr[0],
            self.m.joint(mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "tgt_z")).qposadr[0],
        ]

        # offscreen renderer (always available, for dashboard PNG)
        self.renderer = mujoco.Renderer(self.m, height=720, width=1280)
        self.cam_az   = 160.0
        self.cam_el   = -20.0
        self.cam_dist = 1.2

        # pipeline
        self.pipeline_running = False
        self.pipeline_log     = []
        self.last_ik_err      = 0.0
        self.last_ik_iters    = 0

        # trajectory interpolation for smooth motion
        self.interpolating    = False
        self.interp_start     = None
        self.interp_target    = None
        self.interp_duration  = 2.0  # seconds for smooth motion
        self.interp_start_time = 0.0
        
        # Store last commanded positions to maintain them
        # Initialize from current control values (should be 0 after reset)
        self.last_commanded   = [float(self.d.ctrl[self.act_ids[name]]) for name in ARM_JOINTS]
        print(f"  Initial control values: {self.last_commanded}")

        # viewer handle
        self._viewer_handle = None
        self._use_viewer    = use_viewer
        self._viewer_open   = False

        # start physics + viewer
        self._run = True
        if use_viewer:
            # viewer must be launched from main thread on most platforms;
            # we do it here during construction (called from main thread)
            self._launch_viewer()
        
        # Start physics thread
        self._pth = threading.Thread(target=self._physics_loop, daemon=True)
        self._pth.start()
        
        # Start dedicated viewer sync thread for smooth motion
        if self._use_viewer and self._viewer_open:
            self._vth = threading.Thread(target=self._viewer_sync_loop, daemon=True)
            self._vth.start()
        else:
            print("  No viewer sync thread (headless or viewer failed to open)")

    # ── viewer ───────────────────────────────────────────────────────────────
    def _launch_viewer(self):
        """Launch the passive MuJoCo viewer (non-blocking)."""
        try:
            # IMPORTANT: launch_passive copies mjData internally
            # We must hold the lock to prevent physics loop interference
            with self.lock:
                self._viewer_handle = mujoco.viewer.launch_passive(
                    self.m, self.d,
                    show_left_ui=True,
                    show_right_ui=True,
                )
            self._viewer_open = True
            print("✓ MuJoCo viewer window opened")
            print("  Tip: use the viewer to orbit/zoom; IK commands update it live.")
            
            # Start viewer sync thread if not already running
            if not hasattr(self, '_vth') or not self._vth.is_alive():
                self._vth = threading.Thread(target=self._viewer_sync_loop, daemon=True)
                self._vth.start()
                
        except Exception as ex:
            print(f"⚠  Could not open viewer: {ex}")
            print("   (Running headless — use dashboard PNG stream instead)")
            self._viewer_open = False

    def viewer_is_open(self) -> bool:
        if self._viewer_handle is None:
            return False
        try:
            return self._viewer_handle.is_running()
        except Exception:
            return False

    def _viewer_sync_loop(self):
        """Dedicated thread to sync viewer at 60 FPS for smooth motion."""
        target_fps = 60
        dt = 1.0 / target_fps
        
        print("  Viewer sync thread started (60 FPS)")
        
        while self._run:
            if not self.viewer_is_open():
                break
                
            try:
                # Sync viewer - this internally copies mjData
                # We need to acquire lock briefly to prevent data corruption
                with self.lock:
                    if self._viewer_handle is not None:
                        self._viewer_handle.sync()
            except Exception as e:
                # Viewer might be closing or temporary conflict
                print(f"  Viewer sync error: {e}")
                break
                
            time.sleep(dt)
        
        print("  Viewer sync thread stopped")
    
    def _sync_viewer_manual(self):
        """Manual sync for when viewer sync thread isn't running (headless mode)."""
        if self._viewer_handle is not None and self.viewer_is_open():
            try:
                with self.lock:
                    self._viewer_handle.sync()
            except Exception:
                pass

    # ── physics loop ─────────────────────────────────────────────────────────
    def _physics_loop(self):
        dt = self.m.opt.timestep          # 0.002 s
        while self._run:
            with self.lock:
                # Handle smooth interpolation if active
                if self.interpolating:
                    elapsed = self.d.time - self.interp_start_time
                    t = min(elapsed / self.interp_duration, 1.0)  # clamp to [0, 1]
                    
                    # Smooth interpolation using cubic easing
                    t_smooth = t * t * (3.0 - 2.0 * t)  # smoothstep function
                    
                    # Interpolate between start and target control values
                    for i, name in enumerate(ARM_JOINTS):
                        interpolated = self.interp_start[i] + t_smooth * (self.interp_target[i] - self.interp_start[i])
                        self.d.ctrl[self.act_ids[name]] = interpolated
                        self.last_commanded[i] = interpolated
                    
                    # Stop interpolating when done and HOLD the target position
                    if t >= 1.0:
                        self.interpolating = False
                        # Explicitly set final target values
                        for i, name in enumerate(ARM_JOINTS):
                            self.d.ctrl[self.act_ids[name]] = self.interp_target[i]
                            self.last_commanded[i] = self.interp_target[i]
                        print(f"  → Reached target, holding at {[f'{v:.3f}' for v in self.interp_target]}")
                # Note: When not interpolating, we DON'T continuously write control values
                # The position actuators will naturally hold the last commanded position
                # Continuously writing causes fighting with the actuator's PID controller
                
                mujoco.mj_step(self.m, self.d)
            # Viewer sync thread handles updates at 60 FPS
            time.sleep(dt * 0.5)          # run ~2× real-time

    # ── IK ───────────────────────────────────────────────────────────────────
    def solve_ik(self, x, y, z, smooth=True, duration=2.0, wrist_angle=None):
        """
        Solve IK for target position (x, y, z).
        
        Args:
            x, y, z: Target end-effector position in world coordinates
            smooth: If True, use smooth interpolation; if False, instant move
            duration: Time in seconds for smooth interpolation
            wrist_angle: Optional wrist_roll angle in radians to control gripper rotation.
                        Use π/2 (~1.57) for horizontal gripper fingers (parallel to floor)
                        If None, wrist_roll is determined naturally by IK (usually 0)
        """
        # Get current CONTROL values (not positions - that's the key!)
        # This ensures we start interpolating from where we commanded, not where we are
        with self.lock:
            current_q = [float(self.d.ctrl[self.act_ids[name]]) for name in ARM_JOINTS]
            # Copy ALL qpos and qvel to scratch data for IK solving
            self.d_scratch.qpos[:] = self.d.qpos[:]
            self.d_scratch.qvel[:] = self.d.qvel[:]
            # Update target marker (visual only, on live data)
            self.d.qpos[self.tgt_ids[0]] = x
            self.d.qpos[self.tgt_ids[1]] = y
            self.d.qpos[self.tgt_ids[2]] = z
        
        # Initialize scratch data with full forward pass before solving
        mujoco.mj_forward(self.m, self.d_scratch)
        
        # Solve IK on scratch data (outside lock - doesn't touch live sim)
        q, err, iters = self.ik.solve([x, y, z])
        
        # Apply wrist angle constraint if specified
        if wrist_angle is not None:
            # wrist_roll is the 5th joint (index 4) in ARM_JOINTS
            # ARM_JOINTS = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
            # wrist_roll controls the rotation of the gripper around its axis
            q[4] = wrist_angle
            
            # After modifying wrist angle, recompute error
            # Set the modified q to scratch data
            for k, ji in enumerate(self.ik.jnt_ids):
                self.d_scratch.qpos[self.m.joint(ji).qposadr[0]] = q[k]
            mujoco.mj_kinematics(self.m, self.d_scratch)
            err = float(np.linalg.norm(np.array([x, y, z]) - self.d_scratch.site_xpos[self.ik.ee_id]))
        
        self.last_ik_err   = err
        self.last_ik_iters = iters
        
        print(f"  IK: current_ctrl={[f'{v:.3f}' for v in current_q]}, target={[f'{v:.3f}' for v in q]}" + 
              (f", wrist_angle={wrist_angle:.3f}" if wrist_angle is not None else ""))
        
        # Now set up interpolation or instant move (brief lock)
        with self.lock:
            if smooth:
                # Start smooth interpolation from current control values
                self.interp_start = current_q
                self.interp_target = q
                self.interp_duration = duration
                self.interp_start_time = self.d.time
                self.interpolating = True
                # DO NOT call mj_forward here - let physics loop handle it naturally
            else:
                # Instant jump (old behavior)
                for i, name in enumerate(ARM_JOINTS):
                    self.d.ctrl[self.act_ids[name]] = q[i]
                    self.last_commanded[i] = q[i]
                mujoco.mj_forward(self.m, self.d)
        # Viewer sync thread handles updates automatically
        return q, err, iters

    # ── direct control ────────────────────────────────────────────────────────
    def set_joints(self, joints):
        with self.lock:
            for i, name in enumerate(ALL_ACTS[:len(joints)]):
                self.d.ctrl[self.act_ids[name]] = joints[i]
            # Update last commanded for arm joints
            for i in range(min(len(joints), len(ARM_JOINTS))):
                self.last_commanded[i] = joints[i]
            # Physics loop will handle forward kinematics naturally

    def set_gripper(self, value):
        with self.lock:
            # Gripper range: -0.174 to 1.745 rad
            # We want: value 0 = fully OPEN (1.745 rad)
            #         value 1 = fully CLOSED (-0.174 rad for tight grip)
            # Map [0, 1] to [1.745, -0.174]
            gripper_angle = 1.7453 - value * (1.7453 + 0.17453)
            self.d.ctrl[self.act_ids["gripper"]] = gripper_angle
            # Physics loop will handle forward kinematics naturally

    def home(self):
        with self.lock:
            for i, name in enumerate(ALL_ACTS):
                self.d.ctrl[self.act_ids[name]] = 0.0
            # Update last commanded for arm joints
            for i in range(len(ARM_JOINTS)):
                self.last_commanded[i] = 0.0
            # Physics loop will handle forward kinematics naturally

    # ── pipeline ──────────────────────────────────────────────────────────────
    def run_pipeline(self, steps):
        def _exec():
            self.pipeline_running = True
            self.pipeline_log     = []
            for i, step in enumerate(steps):
                stype = step.get("type", "move_ik")
                label = step.get("label", f"Step {i+1}")
                self.pipeline_log.append(f"▶ {label}")
                if stype == "move_ik":
                    x, y, z = step["x"], step["y"], step["z"]
                    # Get motion duration (default 2s) and settle time
                    duration = step.get("duration", 2.0)
                    settle_time = step.get("settle_time", 0.5)
                    # Get optional wrist angle for gripper orientation control
                    wrist_angle = step.get("wrist_angle", None)
                    
                    # Start the motion
                    q, err, iters = self.solve_ik(x, y, z, smooth=True, duration=duration, wrist_angle=wrist_angle)
                                        # Mirror solved joints to hardware bridge
                    mirror_hw("/move", {
                        "joints": {
                            "shoulder_pan": q[0],
                            "shoulder_lift": q[1],
                            "elbow_flex": q[2],
                            "wrist_flex": q[3],
                            "wrist_roll": q[4]
                        }
                    })
                    
                    # Wait for interpolation to complete
                    time.sleep(duration)
                    
                    # Wait for actuators to settle at target
                    time.sleep(settle_time)
                    
                    self.pipeline_log.append(f"  ✓ err={err*1000:.1f}mm iters={iters} ({duration}s motion + {settle_time}s settle)")
                elif stype == "gripper":
                    self.set_gripper(step.get("value", 0))
                    mirror_hw("/gripper", {
                        "value": step.get("value", 0)
                    })
                    time.sleep(step.get("settle_time", 0.8))
                    # Now inverted: value 0 = OPEN, value 1 = CLOSE
                    self.pipeline_log.append(f"  ✓ Gripper={'OPEN' if step.get('value',0)<0.5 else 'CLOSE'}")
                elif stype == "home":
                    self.home()
                    mirror_hw("/home", {})
                    time.sleep(1.5)
                    self.pipeline_log.append("  ✓ Home pose")
                elif stype == "wait":
                    t = step.get("duration", 1.0)
                    time.sleep(t)
                    self.pipeline_log.append(f"  ✓ Waited {t}s")
            self.pipeline_running = False
            self.pipeline_log.append("✅ Pipeline complete")
        threading.Thread(target=_exec, daemon=True).start()

    # ── state / render ────────────────────────────────────────────────────────
    def get_state(self):
        with self.lock:
            # Ensure forward kinematics is complete before reading
            mujoco.mj_forward(self.m, self.d)
            ee   = self.d.site_xpos[self.ee_id].copy()
            q    = {n: float(self.d.qpos[self.m.joint(self.jnt_ids[n]).qposadr[0]])
                    for n in list(ARM_JOINTS) + ["gripper"]}
            ctrl = {n: float(self.d.ctrl[self.act_ids[n]]) for n in ALL_ACTS}
        return {
            "ee": ee.tolist(), "joints": q, "ctrl": ctrl,
            "ik_err_mm":   round(self.last_ik_err * 1000, 2),
            "ik_iters":    self.last_ik_iters,
            "pipeline_running": self.pipeline_running,
            "pipeline_log":     self.pipeline_log[-30:],
            "viewer_open":      self.viewer_is_open(),
        }

    def render_png(self) -> str:
        # The renderer's update_scene can conflict with physics loop
        # Strategy: acquire lock briefly, copy only what we need, then render
        with self.lock:
            # Shallow copy of the data we need for rendering
            qpos_copy = self.d.qpos.copy()
            qvel_copy = self.d.qvel.copy()
            ctrl_copy = self.d.ctrl.copy()
        
        # Create temporary mjData outside the lock
        d_render = mujoco.MjData(self.m)
        d_render.qpos[:] = qpos_copy
        d_render.qvel[:] = qvel_copy
        d_render.ctrl[:] = ctrl_copy
        mujoco.mj_forward(self.m, d_render)
        
        # Render using the temporary data (no lock needed)
        cam = mujoco.MjvCamera()
        cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
        cam.azimuth   = self.cam_az
        cam.elevation = self.cam_el
        cam.distance  = self.cam_dist
        cam.lookat[:] = [0.1, 0.0, 0.22]
        self.renderer.update_scene(d_render, camera=cam)
        px = self.renderer.render()
        
        buf = io.BytesIO()
        Image.fromarray(px).save(buf, format="PNG")
        return base64.b64encode(buf.getvalue()).decode()


# ══════════════════════════════════════════════════════════════════════════════
#  HTTP Server
# ══════════════════════════════════════════════════════════════════════════════
SIM: SimState = None   # set in main()

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass   # quiet

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def send_json(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def read_json(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n)) if n else {}

    def do_OPTIONS(self):
        self.send_response(200); self._cors(); self.end_headers()

    def do_GET(self):
        if self.path == "/state":
            self.send_json(SIM.get_state())
        elif self.path == "/render":
            self.send_json({"png": SIM.render_png()})
        elif self.path == "/viewer_status":
            self.send_json({"open": SIM.viewer_is_open()})
        else:
            self.send_json({"error": "not found"}, 404)

    def do_POST(self):
        body = self.read_json()
        p = self.path

        if p == "/ik":
            x, y, z = body["x"], body["y"], body["z"]
            # Get smooth motion parameters (default: smooth=True, 2 seconds)
            smooth = body.get("smooth", True)
            duration = body.get("duration", 2.0)
            wrist_angle = body.get("wrist_angle", None)
            q, err, iters = SIM.solve_ik(x, y, z, smooth=smooth, duration=duration, wrist_angle=wrist_angle)
            state = SIM.get_state()
            png   = SIM.render_png()
            self.send_json({"joints": dict(zip(ARM_JOINTS, q)),
                            "err_mm": round(err * 1000, 2),
                            "iters": iters, "state": state, "png": png})

        elif p == "/move":
            SIM.set_joints(body["joints"])
            self.send_json({"ok": True})

        elif p == "/gripper":
            SIM.set_gripper(body.get("value", 0))
            self.send_json({"ok": True})

        elif p == "/pipeline":
            SIM.run_pipeline(body["steps"])
            self.send_json({"ok": True, "msg": "Pipeline started"})

        elif p == "/home":
            SIM.home()
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

    ap = argparse.ArgumentParser(description="SO-ARM101 IK Simulation Server")
    ap.add_argument("--viewer", action="store_true",
                    help="Open the live MuJoCo 3-D viewer window")
    ap.add_argument("--port", type=int, default=PORT,
                    help=f"HTTP port (default {PORT})")
    ap.add_argument("--scene", default=SCENE,
                    help="Path to scene XML")
    args = ap.parse_args()

    print("=" * 60)
    print("  SO-ARM101  IK Server  (MuJoCo 3)")
    print("=" * 60)
    if args.viewer:
        print("  Mode: LIVE VIEWER  (interactive 3-D window)")
    else:
        print("  Mode: HEADLESS  (PNG stream to dashboard)")
        print("  Tip : run with --viewer to open the 3-D window")
    print()

    # Build SimState — viewer launched here (must be main thread on Linux/macOS)
    SIM = SimState(use_viewer=args.viewer)
    ee0 = SIM.d.site_xpos[SIM.ee_id]
    print(f"  EE home pos : [{ee0[0]:.4f}, {ee0[1]:.4f}, {ee0[2]:.4f}]")
    print(f"  HTTP port   : {args.port}")
    print()

    srv = HTTPServer(("127.0.0.1", args.port), Handler)
    print(f"✓ Server ready → http://127.0.0.1:{args.port}")
    print("  Open so101_dashboard.html in your browser and click Connect.")
    if args.viewer:
        print("  The MuJoCo viewer is live — orbit with mouse, IK updates it in real-time.")
    print()

    # HTTP loop (blocking) — viewer is driven by sync() calls from physics thread
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        SIM._run = False
        if SIM._viewer_handle is not None:
            try: SIM._viewer_handle.close()
            except: pass


if __name__ == "__main__":
    main()
