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
  POST /coordinate_action { x, y, z, pre_gripper?, post_gripper?, approach_z?, wrist_angle?, orientation?, retreat? }
  POST /coordinate_preview { x, y, z, pre_gripper?, post_gripper?, approach_z?, wrist_angle?, orientation? }
  POST /radius_limits  { z, wrist_angle?, orientation?, max_err_mm? }
  POST /move          { joints: [6 floats] }
  POST /gripper       { value: 0..1 }
  POST /pipeline      { steps: [...] }
  POST /setcam        { az, el, dist }
  POST /home
  POST /stop
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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np

# ── GL backend: use EGL for offscreen; viewer manages its own context ────────
os.environ.setdefault("MUJOCO_GL", "wgl")
import mujoco
import mujoco.viewer
from PIL import Image

BASE_DIR   = os.path.dirname(__file__)
SCENE      = os.path.join(BASE_DIR, "scene_full.xml")
COORD_UI   = os.path.join(BASE_DIR, "coordinate_control.html")
HOME_POSE_FILE = os.path.join(BASE_DIR, "home_pose.json")
GRIPPER_POSITIONS_FILE = os.path.join(BASE_DIR, "gripper_positions.json")
ARM_JOINTS = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ALL_ACTS   = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
EE_SITE    = "gripperframe"
PORT       = 7842
MIN_TARGET_RADIUS = 0.12
X_RANGE = (-0.5, 0.6)
Y_RANGE = (-0.5, 0.5)
Z_RANGE = (-0.05, 0.65)
WRIST_ROLL_RANGE = (-2.74385, 2.84121)
# Close slightly past the MuJoCo model's nominal lower limit so the real gripper
# clamps fully; the sim joint itself will still respect its physical joint range.
GRIPPER_CLOSED_ANGLE = -0.55
GRIPPER_FULL_OPEN_ANGLE = 1.7453
GRIPPER_OPEN_FRACTION = 0.35
DEFAULT_GRIPPER_POSITIONS = {
    "open": GRIPPER_CLOSED_ANGLE + GRIPPER_OPEN_FRACTION * (GRIPPER_FULL_OPEN_ANGLE - GRIPPER_CLOSED_ANGLE),
    "close": GRIPPER_CLOSED_ANGLE,
}
IK_SEEDS = [
    [0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.6, -1.0, 0.3, 0.0],
    [0.0, -0.6, 1.0, -0.3, 0.0],
    [0.0, 1.0, -1.3, -0.2, 0.0],
    [0.0, -0.2, -1.2, 1.0, 0.0],
]

# Folded rest pose used by /home. Angles are radians and kept within scene ctrlrange.
# Gripper uses raw actuator angle here: closed is the physical closed limit.
HOME_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 1.45,
    "elbow_flex": -1.45,
    "wrist_flex": -1.20,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_CLOSED_ANGLE,
}

def gripper_value_to_angle(value):
    """Map UI/API gripper value where 0=part-open and 1=fully closed."""
    positions = load_gripper_positions()
    value = max(0.0, min(1.0, float(value)))
    return positions["open"] + value * (positions["close"] - positions["open"])

def _validated_gripper_positions(positions):
    validated = {}
    for name in ("open", "close"):
        if name not in positions:
            raise ValueError(f"Missing gripper position: {name}")
        value = float(positions[name])
        if not math.isfinite(value):
            raise ValueError(f"Invalid gripper position for {name}")
        validated[name] = value
    return validated

def load_gripper_positions():
    try:
        with open(GRIPPER_POSITIONS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return _validated_gripper_positions(data.get("positions", data))
    except FileNotFoundError:
        return DEFAULT_GRIPPER_POSITIONS.copy()
    except Exception as e:
        print(f"  Could not load gripper positions: {e}")
        return DEFAULT_GRIPPER_POSITIONS.copy()

def save_gripper_positions(positions):
    positions = _validated_gripper_positions(positions)
    payload = {
        "positions": positions,
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    with open(GRIPPER_POSITIONS_FILE, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    return positions

def _validated_home_pose(pose):
    validated = {}
    for name in ALL_ACTS:
        if name not in pose:
            raise ValueError(f"Missing home pose joint: {name}")
        value = float(pose[name])
        if not math.isfinite(value):
            raise ValueError(f"Invalid home pose value for {name}")
        validated[name] = value
    return validated

def parse_gripper_value(value, field_name="gripper"):
    if value is None:
        return None
    if isinstance(value, str):
        grip = value.strip().lower()
        if grip in ("close", "closed", "grip", "1", "true"):
            return 1.0
        if grip in ("open", "opened", "release", "0", "false"):
            return 0.0
        if grip in ("hold", "none", ""):
            return None
        raise ValueError(f"{field_name} must be open, close, hold, or a 0..1 value")
    grip_value = float(value)
    if not (0.0 <= grip_value <= 1.0):
        raise ValueError(f"{field_name} value must be between 0 and 1")
    return grip_value

def parse_orientation(value):
    if value is None:
        return "free"
    orientation = str(value).strip().lower()
    aliases = {
        "none": "free",
        "natural": "free",
        "free": "free",
        "down": "downwards",
        "downward": "downwards",
        "downwards": "downwards",
        "horizontal": "horizontal",
        "sideways": "horizontal",
    }
    if orientation not in aliases:
        raise ValueError("orientation must be free, downwards, or horizontal")
    return aliases[orientation]

def orientation_axis_for(mode, x, y):
    if mode == "downwards":
        return np.array([0.0, 0.0, -1.0])
    if mode == "horizontal":
        radius = math.hypot(x, y)
        if radius < 1e-6:
            return np.array([1.0, 0.0, 0.0])
        return np.array([x / radius, y / radius, 0.0])
    return None

def load_home_pose():
    try:
        with open(HOME_POSE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return _validated_home_pose(data.get("pose", data))
    except FileNotFoundError:
        return HOME_POSE.copy()
    except Exception as e:
        print(f"  Could not load custom home pose: {e}")
        return HOME_POSE.copy()

def save_home_pose(pose):
    pose = _validated_home_pose(pose)
    payload = {
        "pose": pose,
        "joint_order": ALL_ACTS,
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    with open(HOME_POSE_FILE, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    return pose

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

    def _jacobian(self, active_indices=None):
        Jp = np.zeros((3, self.m.nv)); Jr = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, self.d, Jp, Jr, self.ee_id)
        dof_ids = self.dof_ids if active_indices is None else [self.dof_ids[i] for i in active_indices]
        return Jp[:, dof_ids], Jr[:, dof_ids]

    def solve(self, target, locked=None, target_axis=None, axis_index=0, orientation_weight=0.08):
        q = self._get_q().copy()
        locked = locked or {}
        active_indices = [i for i in range(len(self.jnt_ids)) if i not in locked]
        target_axis = None if target_axis is None else np.asarray(target_axis, dtype=float)
        if target_axis is not None:
            norm = float(np.linalg.norm(target_axis))
            if norm > 1e-9:
                target_axis = target_axis / norm
            else:
                target_axis = None
        for idx, value in locked.items():
            q[idx] = value
        for it in range(self.max_iter):
            for idx, value in locked.items():
                q[idx] = value
            self._set_q(q)
            mujoco.mj_kinematics(self.m, self.d)
            pos_err = np.array(target) - self.d.site_xpos[self.ee_id]
            solve_err = pos_err
            Jp, Jr = self._jacobian(active_indices)
            J = Jp
            axis_err_norm = 0.0
            if target_axis is not None:
                site_xmat = self.d.site_xmat[self.ee_id].reshape(3, 3)
                current_axis = site_xmat[:, axis_index]
                axis_err = np.cross(current_axis, target_axis)
                axis_err_norm = float(np.linalg.norm(axis_err))
                solve_err = np.concatenate([pos_err, orientation_weight * axis_err])
                J = np.vstack([Jp, orientation_weight * Jr])
            en = float(np.linalg.norm(pos_err))
            if en < self.tol and axis_err_norm < 0.03:
                break
            JJt = J @ J.T
            lam = self.lam if float(np.linalg.det(JJt)) > 1e-8 else self.lam * 20
            dq_active = self.alpha * J.T @ np.linalg.solve(JJt + lam**2 * np.eye(JJt.shape[0]), solve_err)
            q_next = q.copy()
            for local_idx, q_idx in enumerate(active_indices):
                q_next[q_idx] += dq_active[local_idx]
            q = self._clip(q_next)
            for idx, value in locked.items():
                q[idx] = value
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
        self.pipeline_stop_requested = False
        self.pipeline_log     = []
        self.last_ik_err      = 0.0
        self.last_ik_iters    = 0
        self.home_pose        = load_home_pose()
        self.gripper_positions = load_gripper_positions()
        self.radius_limit_cache = {}

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
                    
                    # Quintic smootherstep: zero velocity and acceleration at both ends.
                    t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
                    
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
    def _solve_best_ik(self, x, y, z, wrist_angle=None, orientation="free"):
        """Try IK from the current pose and several neutral seeds, then keep the lowest-error solution."""
        with self.lock:
            qpos_snapshot = self.d.qpos.copy()
            qvel_snapshot = self.d.qvel.copy()
            current_q = [float(self.d.ctrl[self.act_ids[name]]) for name in ARM_JOINTS]

        target = np.array([x, y, z], dtype=float)
        target_axis = orientation_axis_for(orientation, x, y)
        base_yaw = math.atan2(y, x) if abs(x) > 1e-9 or abs(y) > 1e-9 else 0.0
        seeds = [current_q]
        for seed in IK_SEEDS:
            seeded = seed.copy()
            seeded[0] = base_yaw
            seeds.append(seeded)

        best = None
        best_score = None
        scratch = mujoco.MjData(self.m)
        local_ik = DLSIK(self.m, scratch, EE_SITE, ARM_JOINTS)
        for seed in seeds:
            scratch.qpos[:] = qpos_snapshot
            scratch.qvel[:] = qvel_snapshot
            if wrist_angle is not None:
                seed[4] = wrist_angle
            for k, ji in enumerate(local_ik.jnt_ids):
                scratch.qpos[self.m.joint(ji).qposadr[0]] = seed[k]
            mujoco.mj_forward(self.m, scratch)

            locked = {4: wrist_angle} if wrist_angle is not None else None
            q, err, iters = local_ik.solve(target, locked=locked, target_axis=target_axis)

            axis_err = 0.0
            if target_axis is not None:
                site_xmat = scratch.site_xmat[local_ik.ee_id].reshape(3, 3)
                current_axis = site_xmat[:, 0]
                axis_err = float(np.linalg.norm(np.cross(current_axis, target_axis)))
            score = err + 0.08 * axis_err
            if best is None or score < best_score:
                best = (q, err, iters)
                best_score = score

        return best

    def solve_ik(self, x, y, z, smooth=True, duration=2.0, wrist_angle=None, orientation="free"):
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
        with self.lock:
            current_q = [float(self.d.ctrl[self.act_ids[name]]) for name in ARM_JOINTS]
            # Update target marker (visual only, on live data)
            self.d.qpos[self.tgt_ids[0]] = x
            self.d.qpos[self.tgt_ids[1]] = y
            self.d.qpos[self.tgt_ids[2]] = z

        orientation = parse_orientation(orientation)
        q, err, iters = self._solve_best_ik(x, y, z, wrist_angle=wrist_angle, orientation=orientation)
        
        self.last_ik_err   = err
        self.last_ik_iters = iters
        
        print(f"  IK: current_ctrl={[f'{v:.3f}' for v in current_q]}, target={[f'{v:.3f}' for v in q]}" + 
              (f", wrist_angle={wrist_angle:.3f}" if wrist_angle is not None else "") +
              (f", orientation={orientation}" if orientation != "free" else ""))
        
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

    def preview_ik(self, x, y, z, wrist_angle=None, orientation="free"):
        """Solve IK against scratch data without moving the live simulation."""
        orientation = parse_orientation(orientation)
        return self._solve_best_ik(x, y, z, wrist_angle=wrist_angle, orientation=orientation)

    def build_coordinate_action(self, command):
        """Expand a Cartesian target and gripper state into a safe mini-pipeline."""
        if self.pipeline_running:
            raise ValueError("A pipeline is already running")

        x = float(command["x"])
        y = float(command["y"])
        z = float(command["z"])
        duration = float(command.get("duration", 1.5))
        settle_time = float(command.get("settle_time", 0.6))
        gripper_settle = float(command.get("gripper_settle", 0.8))
        max_err_mm = float(command.get("max_err_mm", 15.0))
        orientation = parse_orientation(command.get("orientation", command.get("gripper_orientation", "free")))
        wrist_angle = command.get("wrist_angle", None)
        if wrist_angle is not None:
            wrist_angle = float(wrist_angle)
            if not (WRIST_ROLL_RANGE[0] <= wrist_angle <= WRIST_ROLL_RANGE[1]):
                raise ValueError(
                    f"wrist_angle must be between {WRIST_ROLL_RANGE[0]:.2f} and "
                    f"{WRIST_ROLL_RANGE[1]:.2f} radians"
                )

        values = [x, y, z, duration, settle_time, gripper_settle, max_err_mm]
        if wrist_angle is not None:
            values.append(wrist_angle)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("All numeric fields must be finite")
        if not (X_RANGE[0] <= x <= X_RANGE[1] and Y_RANGE[0] <= y <= Y_RANGE[1] and Z_RANGE[0] <= z <= Z_RANGE[1]):
            raise ValueError("Target is outside the configured XYZ bounds")
        radius = math.hypot(x, y)
        if radius < MIN_TARGET_RADIUS:
            raise ValueError(
                f"Target is too close to the base: radial distance {radius:.3f}m is below "
                f"{MIN_TARGET_RADIUS:.3f}m. Increase X or Y."
            )
        if not (0.2 <= duration <= 10.0 and 0.0 <= settle_time <= 5.0 and 0.0 <= gripper_settle <= 5.0):
            raise ValueError("Timing values are outside the allowed range")
        if not (1.0 <= max_err_mm <= 100.0):
            raise ValueError("max_err_mm must be between 1 and 100")

        requested_approach_z = command.get("approach_z", None)
        if requested_approach_z is None:
            approach_z = min(max(z + 0.08, 0.12), Z_RANGE[1])
        else:
            approach_z = float(requested_approach_z)
            if not math.isfinite(approach_z):
                raise ValueError("approach_z must be finite")
        approach_z = max(approach_z, z)
        if not (Z_RANGE[0] <= approach_z <= Z_RANGE[1]):
            raise ValueError("Approach height is outside the configured Z bounds")

        pre_grip_value = parse_gripper_value(command.get("pre_gripper", "hold"), "pre_gripper")
        post_gripper = command.get("post_gripper", command.get("gripper", command.get("gripper_action", "hold")))
        post_grip_value = parse_gripper_value(command.get("gripper_value", post_gripper), "post_gripper")

        preflight = []
        failed_checks = []
        for label, px, py, pz in (
            ("Approach target", x, y, approach_z),
            ("Move to coordinate", x, y, z),
        ):
            _, err, iters = self.preview_ik(px, py, pz, wrist_angle=wrist_angle, orientation=orientation)
            err_mm = err * 1000.0
            preflight.append({"label": label, "err_mm": round(err_mm, 2), "iters": iters})
            if err_mm > max_err_mm:
                failed_checks.append(f"{label} IK error {err_mm:.1f}mm exceeds {max_err_mm:.1f}mm")

        if failed_checks:
            error = "; ".join(failed_checks)
            limits = self.radius_limits_for_z(z, wrist_angle=wrist_angle, orientation=orientation, max_err_mm=max_err_mm)
            if limits["min_radius"] is None:
                hint = (
                    f"No radius solved at Z={z:.3f}m with the current wrist angle. "
                    "Try increasing Z or setting wrist roll to 0."
                )
            elif radius < limits["min_radius"]:
                hint = (
                    f"At Z={z:.3f}m, usable radius is about "
                    f"{limits['min_radius']:.3f}m to {limits['max_radius']:.3f}m. Move farther from the base."
                )
            elif radius > limits["max_radius"]:
                hint = (
                    f"At Z={z:.3f}m, usable radius is about "
                    f"{limits['min_radius']:.3f}m to {limits['max_radius']:.3f}m. Reduce X/Y reach."
                )
            else:
                hint = "Try increasing Z, lowering max speed/duration changes, or setting wrist roll to 0."
            raise ValueError(f"{error}. {hint}")

        steps = []
        if pre_grip_value is not None:
            steps.append({
                "type": "gripper",
                "label": "Open gripper before move" if pre_grip_value < 0.5 else "Close gripper before move",
                "value": pre_grip_value,
                "settle_time": gripper_settle,
            })
        steps.extend([
            {
                "type": "move_ik",
                "label": "Approach target",
                "x": x,
                "y": y,
                "z": approach_z,
                "orientation": orientation,
                "duration": duration,
                "settle_time": settle_time,
            },
            {
                "type": "move_ik",
                "label": "Move to coordinate",
                "x": x,
                "y": y,
                "z": z,
                "orientation": orientation,
                "duration": duration,
                "settle_time": settle_time,
            },
        ])
        if wrist_angle is not None:
            for step in steps:
                if step["type"] == "move_ik":
                    step["wrist_angle"] = wrist_angle
        if post_grip_value is not None:
            steps.append({
                "type": "gripper",
                "label": "Close gripper after reaching target" if post_grip_value >= 0.5 else "Open gripper after reaching target",
                "value": post_grip_value,
                "settle_time": gripper_settle,
            })
        if command.get("retreat", True):
            retreat = {
                "type": "move_ik",
                "label": "Retreat above target",
                "x": x,
                "y": y,
                "z": approach_z,
                "orientation": orientation,
                "duration": duration,
                "settle_time": settle_time,
            }
            if wrist_angle is not None:
                retreat["wrist_angle"] = wrist_angle
            steps.append(retreat)

        return steps, preflight

    def radius_limits_for_z(self, z, wrist_angle=None, orientation="free", max_err_mm=15.0):
        z = float(z)
        max_err_mm = float(max_err_mm)
        orientation = parse_orientation(orientation)
        if wrist_angle is not None:
            wrist_angle = float(wrist_angle)
            if not (WRIST_ROLL_RANGE[0] <= wrist_angle <= WRIST_ROLL_RANGE[1]):
                raise ValueError(
                    f"wrist_angle must be between {WRIST_ROLL_RANGE[0]:.2f} and "
                    f"{WRIST_ROLL_RANGE[1]:.2f} radians"
                )
        values = [z, max_err_mm]
        if wrist_angle is not None:
            values.append(wrist_angle)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("All numeric fields must be finite")
        if not (Z_RANGE[0] <= z <= Z_RANGE[1]):
            raise ValueError("Z is outside the configured bounds")
        if not (1.0 <= max_err_mm <= 100.0):
            raise ValueError("max_err_mm must be between 1 and 100")

        cache_key = (round(z, 3), None if wrist_angle is None else round(wrist_angle, 2), orientation, round(max_err_mm, 1))
        if cache_key in self.radius_limit_cache:
            return self.radius_limit_cache[cache_key].copy()

        reachable = []
        for radius in np.linspace(MIN_TARGET_RADIUS, 0.6, 25):
            _, err, _ = self.preview_ik(float(radius), 0.0, z, wrist_angle=wrist_angle, orientation=orientation)
            if err * 1000.0 <= max_err_mm:
                reachable.append(float(radius))

        if not reachable:
            result = {"z": z, "min_radius": None, "max_radius": None}
        else:
            result = {
                "z": z,
                "min_radius": round(min(reachable), 3),
                "max_radius": round(max(reachable), 3),
            }
        self.radius_limit_cache[cache_key] = result
        return result.copy()

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
            value = max(0.0, min(1.0, float(value)))
            gripper_angle = self.gripper_positions["open"] + value * (
                self.gripper_positions["close"] - self.gripper_positions["open"]
            )
            self.d.ctrl[self.act_ids["gripper"]] = gripper_angle
            # Physics loop will handle forward kinematics naturally

    def home(self, duration=3.0):
        with self.lock:
            self.interpolating = False
            current_q = [float(self.d.ctrl[self.act_ids[name]]) for name in ARM_JOINTS]
            target_q = [self.home_pose[name] for name in ARM_JOINTS]
            self.interp_start = current_q
            self.interp_target = target_q
            self.interp_duration = max(0.2, float(duration))
            self.interp_start_time = self.d.time
            self.interpolating = True
            self.d.ctrl[self.act_ids["gripper"]] = self.home_pose["gripper"]
            # Physics loop will handle forward kinematics naturally

    def stop(self):
        with self.lock:
            mujoco.mj_forward(self.m, self.d)
            self.interpolating = False
            for i, name in enumerate(ARM_JOINTS):
                current = float(self.d.qpos[self.m.joint(self.jnt_ids[name]).qposadr[0]])
                self.d.ctrl[self.act_ids[name]] = current
                self.last_commanded[i] = current
            self.pipeline_stop_requested = True
            self.pipeline_running = False
            self.pipeline_log.append("STOP requested")

    def current_pose(self):
        with self.lock:
            mujoco.mj_forward(self.m, self.d)
            return {
                name: float(self.d.qpos[self.m.joint(self.jnt_ids[name]).qposadr[0]])
                for name in ALL_ACTS
            }

    def set_home_pose(self, pose=None):
        if pose is None:
            pose = self.current_pose()
        self.home_pose = save_home_pose(pose)
        return self.home_pose.copy()

    def current_gripper_position(self):
        with self.lock:
            mujoco.mj_forward(self.m, self.d)
            return float(self.d.qpos[self.m.joint(self.jnt_ids["gripper"]).qposadr[0]])

    def set_gripper_position(self, name, value=None):
        name = str(name).strip().lower()
        if name not in ("open", "close"):
            raise ValueError("Gripper position name must be open or close")
        if value is None:
            value = self.current_gripper_position()
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("Gripper position value must be finite")
        positions = self.gripper_positions.copy()
        positions[name] = value
        self.gripper_positions = save_gripper_positions(positions)
        return self.gripper_positions.copy()

    # ── pipeline ──────────────────────────────────────────────────────────────
    def _sleep_interruptible(self, duration, interval=0.05):
        end_time = time.time() + max(0.0, float(duration))
        while time.time() < end_time:
            if self.pipeline_stop_requested:
                raise RuntimeError("Stopped by kill switch")
            time.sleep(min(interval, end_time - time.time()))

    def run_pipeline(self, steps):
        if self.pipeline_running:
            raise ValueError("A pipeline is already running")
        self.pipeline_running = True
        self.pipeline_stop_requested = False
        self.pipeline_log     = []

        def _exec():
            try:
                for i, step in enumerate(steps):
                    if self.pipeline_stop_requested:
                        raise RuntimeError("Stopped by kill switch")
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
                        orientation = step.get("orientation", "free")
                        
                        # Start the motion
                        q, err, iters = self.solve_ik(
                            x, y, z,
                            smooth=True,
                            duration=duration,
                            wrist_angle=wrist_angle,
                            orientation=orientation,
                        )
                                            # Mirror solved joints to hardware bridge
                        mirror_hw("/set_motion_time", {"time_ms": int(duration * 1000)})
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
                        self._sleep_interruptible(duration)
                        
                        # Wait for actuators to settle at target
                        self._sleep_interruptible(settle_time)
                        
                        self.pipeline_log.append(f"  ✓ err={err*1000:.1f}mm iters={iters} ({duration}s motion + {settle_time}s settle)")
                    elif stype == "gripper":
                        mirror_hw("/set_motion_time", {"time_ms": int(step.get("settle_time", 0.8) * 1000)})
                        self.set_gripper(step.get("value", 0))
                        mirror_hw("/gripper", {
                            "value": step.get("value", 0)
                        })
                        self._sleep_interruptible(step.get("settle_time", 0.8))
                        # Now inverted: value 0 = OPEN, value 1 = CLOSE
                        self.pipeline_log.append(f"  ✓ Gripper={'OPEN' if step.get('value',0)<0.5 else 'CLOSE'}")
                    elif stype == "home":
                        duration = step.get("duration", 3.0)
                        self.home(duration=duration)
                        mirror_hw("/set_motion_time", {"time_ms": int(duration * 1000)})
                        mirror_hw("/home", {"pose": self.home_pose})
                        self._sleep_interruptible(duration)
                        self.pipeline_log.append("  ✓ Home pose")
                    elif stype == "wait":
                        t = step.get("duration", 1.0)
                        self._sleep_interruptible(t)
                        self.pipeline_log.append(f"  ✓ Waited {t}s")
                self.pipeline_log.append("✅ Pipeline complete")
            except Exception as e:
                self.pipeline_log.append(f"✗ Pipeline failed: {e}")
            finally:
                self.pipeline_running = False
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
        if self.path in ("/", "/coordinate_control.html"):
            try:
                with open(COORD_UI, "rb") as f:
                    body = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self._cors()
                self.end_headers()
                self.wfile.write(body)
            except OSError as e:
                self.send_json({"error": f"coordinate UI not found: {e}"}, 404)
        elif self.path == "/state":
            self.send_json(SIM.get_state())
        elif self.path == "/render":
            self.send_json({"png": SIM.render_png()})
        elif self.path == "/viewer_status":
            self.send_json({"open": SIM.viewer_is_open()})
        elif self.path == "/home_pose":
            self.send_json({"pose": SIM.home_pose, "file": HOME_POSE_FILE})
        elif self.path == "/gripper_positions":
            self.send_json({"positions": SIM.gripper_positions, "file": GRIPPER_POSITIONS_FILE})
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
            orientation = body.get("orientation", body.get("gripper_orientation", "free"))
            q, err, iters = SIM.solve_ik(
                x, y, z,
                smooth=smooth,
                duration=duration,
                wrist_angle=wrist_angle,
                orientation=orientation,
            )
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
            try:
                SIM.run_pipeline(body["steps"])
            except ValueError as e:
                self.send_json({"ok": False, "error": str(e)}, 409)
                return
            self.send_json({"ok": True, "msg": "Pipeline started"})

        elif p == "/coordinate_action":
            try:
                steps, preflight = SIM.build_coordinate_action(body)
            except (KeyError, TypeError, ValueError) as e:
                status = 409 if "already running" in str(e) else 400
                self.send_json({"ok": False, "error": str(e)}, status)
                return
            try:
                SIM.run_pipeline(steps)
            except ValueError as e:
                self.send_json({"ok": False, "error": str(e)}, 409)
                return
            self.send_json({
                "ok": True,
                "msg": "Coordinate action started",
                "steps": steps,
                "preflight": preflight,
            })

        elif p == "/coordinate_preview":
            try:
                steps, preflight = SIM.build_coordinate_action(body)
            except (KeyError, TypeError, ValueError) as e:
                status = 409 if "already running" in str(e) else 400
                self.send_json({"ok": False, "error": str(e)}, status)
                return
            self.send_json({
                "ok": True,
                "msg": "Coordinate action is reachable",
                "steps": steps,
                "preflight": preflight,
            })

        elif p == "/radius_limits":
            try:
                limits = SIM.radius_limits_for_z(
                    body["z"],
                    wrist_angle=body.get("wrist_angle", None),
                    orientation=body.get("orientation", body.get("gripper_orientation", "free")),
                    max_err_mm=body.get("max_err_mm", 15.0),
                )
            except (KeyError, TypeError, ValueError) as e:
                self.send_json({"ok": False, "error": str(e)}, 400)
                return
            self.send_json({"ok": True, **limits})

        elif p == "/home":
            try:
                SIM.run_pipeline([{"type": "home", "label": "Home pose", "duration": 3.0}])
            except ValueError as e:
                self.send_json({"ok": False, "error": str(e)}, 409)
                return
            self.send_json({"ok": True})

        elif p == "/stop":
            SIM.stop()
            mirror_hw("/stop", {})
            self.send_json({"ok": True, "msg": "Stopped"})

        elif p == "/home_pose":
            try:
                pose = body.get("pose") if body else None
                saved = SIM.set_home_pose(pose)
            except (TypeError, ValueError, OSError) as e:
                self.send_json({"ok": False, "error": str(e)}, 400)
                return
            mirror_hw("/home_pose", {"pose": saved})
            self.send_json({"ok": True, "pose": saved, "file": HOME_POSE_FILE})

        elif p == "/gripper_positions":
            try:
                name = body["name"]
                value = body.get("value")
                saved = SIM.set_gripper_position(name, value)
            except (KeyError, TypeError, ValueError, OSError) as e:
                self.send_json({"ok": False, "error": str(e)}, 400)
                return
            mirror_hw("/gripper_positions", {
                "name": str(name).strip().lower(),
                "value": saved[str(name).strip().lower()],
            })
            self.send_json({"ok": True, "positions": saved, "file": GRIPPER_POSITIONS_FILE})

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

    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
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
