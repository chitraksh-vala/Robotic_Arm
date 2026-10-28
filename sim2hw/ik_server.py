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
  GET  /cv_points     → saved named CV calibration points
  GET  /render        → base64 PNG from offscreen renderer (always works)
  POST /ik            { x, y, z, wrist_angle? }
  POST /coordinate_action { x, y, z, pre_gripper?, post_gripper?, approach_z?, wrist_angle?, orientation?, retreat?,
                            max_err_mm?, approach_max_err_mm?, max_axis_err_deg?, approach_max_axis_err_deg?,
                            min_approach_clearance? }
  POST /coordinate_preview { same fields as /coordinate_action }
       Without approach_z, the approach is the highest point (up to max(z+0.15, 0.18))
       reachable straight above the target on the grasp's own arm branch.
       Returned move_ik steps carry the validated "joints" so execution never re-solves.
       Without wrist_angle, orientation=downwards turns wrist_roll so the jaw closing line
       is parallel to robot +X (or jaw_yaw_deg from +X) at the pick position; other
       orientations snap roll to 0/+90/-90 deg. The roll is set on the approach move and
       held for every pose of the action.
  POST /cv_points     { name, board_xy_cm: [x,y], joints?: {...} } records current arm-tip XYZ
  POST /radius_limits  { z, wrist_angle?, orientation?, max_err_mm?, max_axis_err_deg? }
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
CV_POINTS_FILE = os.path.join(BASE_DIR, "cv_robot_points.json")
IK_POSE_POINTS_FILE = os.path.join(BASE_DIR, "ik_pose_points.json")
ARM_JOINTS = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ALL_ACTS   = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
EE_SITE    = "gripperframe"
PORT       = 7842
# Closest allowed target to the base axis. The model predicts the gripper
# touching the shoulder below ~12 cm, but the real arm reaches ~9.7 cm (the
# model is cm-level off there), so contact is only warned about; below ~8 cm
# the approach path cannot be planned anyway. Override with min_radius_m.
MIN_TARGET_RADIUS = 0.08
# Predicted self-contact deeper than this is reported in the preflight.
SELF_CONTACT_WARN_MM = 2.0
X_RANGE = (-0.5, 0.6)
Y_RANGE = (-0.5, 0.5)
Z_RANGE = (-0.05, 0.65)
WRIST_ROLL_RANGE = (-2.74385, 2.84121)
# Downward coordinate actions turn wrist_roll so the line the jaws close
# along is parallel to the robot +X axis (override with jaw_yaw_deg).
DEFAULT_JAW_YAW_DEG = 0.0
# Which side of the object the motor (fixed) jaw goes: base, away or auto.
DEFAULT_FIXED_JAW_SIDE = "base"
MAX_JAW_YAW_ERR_DEG = 5.0
# Other orientations lock wrist_roll to one of these (tried in this order)
# unless the request gives an explicit wrist_angle.
SNAPPED_WRIST_ROLLS = [0.0, math.pi / 2, -math.pi / 2]

def wrap_line_angle(angle):
    """Wrap an angle between two undirected lines into [-pi/2, pi/2)."""
    return (angle + math.pi / 2) % math.pi - math.pi / 2
# How long a coordinate preview's plan is reused by the matching action.
PLAN_CACHE_SECONDS = 120.0
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
    # Extra elbow branches help solve low, close-in targets from the folded home pose.
    [0.0, 1.35, -1.55, 0.55, 0.0],
    [0.0, -1.25, 1.45, -0.45, 0.0],
    [0.0, 0.85, 0.65, -1.25, 0.0],
    [0.0, -0.85, -1.35, 1.25, 0.0],
]

def ik_search_seeds(base_yaw, current_q):
    """Build deterministic elbow/wrist branches for low, close-in targets.

    A single neutral pose often converges to a local minimum for targets near
    the base. Keep the current and hand-tuned seeds, then sample complementary
    shoulder/elbow bends with wrist flex compensating their combined angle.
    """
    seeds = [list(current_q)]
    for seed in IK_SEEDS:
        candidate = seed.copy()
        candidate[0] = base_yaw
        seeds.append(candidate)

    # Keep a compact set of distinct elbow branches. The previous dense grid
    # ran dozens of full 200-iteration solves for every target.
    for shoulder in (-1.25, -0.35, 0.55, 1.35):
        for elbow in (-1.25, 0.0, 1.25):
            # Keep the wrist in a useful range across both folded and extended
            # elbow branches instead of initializing every joint at zero.
            wrist = max(-1.5, min(1.5, -shoulder - elbow))
            seeds.append([base_yaw, shoulder, elbow, wrist, 0.0])
    return seeds

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

def load_cv_points():
    try:
        with open(CV_POINTS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        points = data.get("points", {})
        return points if isinstance(points, dict) else {}
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError, TypeError) as e:
        print(f"  Could not load CV calibration points: {e}")
        return {}

def save_cv_points(points):
    payload = {
        "coordinate_frame": "ik_server_world",
        "units": {"board_xy": "cm", "robot_xyz": "m"},
        "points": points,
    }
    with open(CV_POINTS_FILE, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

def load_ik_pose_points():
    try:
        with open(IK_POSE_POINTS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        points = data.get("points", [])
        if not isinstance(points, list):
            return []
        valid = []
        for point in points:
            xyz = point.get("xyz_m")
            joints = point.get("joints_rad")
            if (isinstance(xyz, list) and len(xyz) == 3 and
                    isinstance(joints, list) and len(joints) == len(ARM_JOINTS)):
                xyz = [float(v) for v in xyz]
                joints = [float(v) for v in joints]
                if all(math.isfinite(v) for v in xyz + joints):
                    valid.append({"xyz_m": xyz, "joints_rad": joints})
        return valid[-128:]
    except FileNotFoundError:
        return []
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as e:
        print(f"  Could not load learned IK poses: {e}")
        return []

def save_ik_pose_points(points):
    payload = {
        "coordinate_frame": "ik_server_world",
        "units": {"xyz": "m", "joints": "rad"},
        "points": points[-128:],
    }
    with open(IK_POSE_POINTS_FILE, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

# ── Optional hardware bridge mirror ──────────────────────────────────────────
HW_BRIDGE_URL = "http://127.0.0.1:7843"

def hw_get(path, timeout=0.5):
    """GET a JSON status from the hardware bridge; None if it is not running."""
    try:
        with urllib.request.urlopen(HW_BRIDGE_URL + path, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except Exception:
        return None


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
# Closed-form 3-vector/3x3 helpers. The solver runs thousands of iterations
# per plan, and numpy's generic cross/solve/norm overhead dominated runtime.
EYE3 = np.eye(3)

def _cross3(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]])

def _norm3(v):
    return math.sqrt(float(v @ v))

def _inv3(m):
    (a, b, c), (d, e, f), (g, h, i) = m.tolist()
    co_a, co_b, co_c = e * i - f * h, f * g - d * i, d * h - e * g
    det = a * co_a + b * co_b + c * co_c
    return np.array([
        [co_a, c * h - b * i, b * f - c * e],
        [co_b, a * i - c * g, c * d - a * f],
        [co_c, b * g - a * h, a * e - b * d],
    ]) / det

class DLSIK:
    def __init__(self, m, d, ee_site, arm_joints,
                 lam=0.015, alpha=0.4, max_iter=200, tol=5e-4):
        self.m, self.d = m, d
        self.lam, self.alpha, self.max_iter, self.tol = lam, alpha, max_iter, tol
        self.ee_id   = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, ee_site)
        self.jnt_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in arm_joints]
        self.dof_ids = [m.joint(ji).dofadr[0] for ji in self.jnt_ids]
        self.qpos_ids = [m.joint(ji).qposadr[0] for ji in self.jnt_ids]
        self.lo = np.array([m.joint(ji).range[0] for ji in self.jnt_ids])
        self.hi = np.array([m.joint(ji).range[1] for ji in self.jnt_ids])

    def _get_q(self):
        return self.d.qpos[self.qpos_ids].copy()

    def _set_q(self, q):
        self.d.qpos[self.qpos_ids] = q

    def _clip(self, q):
        return np.minimum(np.maximum(q, self.lo), self.hi)

    def _jacobian(self, active_indices=None):
        Jp = np.zeros((3, self.m.nv)); Jr = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, self.d, Jp, Jr, self.ee_id)
        dof_ids = self.dof_ids if active_indices is None else [self.dof_ids[i] for i in active_indices]
        return Jp[:, dof_ids], Jr[:, dof_ids]

    def _priority_step(self, Jp, Jr_perp, pos_err, axis_err, free):
        """Task-priority DLS step: position first, tool axis in its null space.

        The 5-DOF arm cannot always satisfy XYZ and a pointing direction at
        once. A weighted solve then silently trades centimetres of position
        error for orientation; here orientation only uses leftover freedom.
        Joints marked not-free (pinned at a limit) are excluded from both tasks.
        """
        n = Jp.shape[1]
        Jp = Jp * free
        JJt = Jp @ Jp.T
        dq_pos = Jp.T @ (_inv3(JJt + self.lam**2 * EYE3) @ pos_err)
        step_norm = math.sqrt(float(dq_pos @ dq_pos))
        if step_norm > 0.25:
            dq_pos *= 0.25 / step_norm
        if Jr_perp is None:
            return dq_pos
        Jr_perp = Jr_perp * free
        # Use an exact projector here: the damped inverse leaks orientation
        # motion back into XYZ, which is exactly what this step must avoid.
        null = np.eye(n) - Jp.T @ (_inv3(JJt + 1e-8 * EYE3) @ Jp)
        Jr_hat = Jr_perp @ null
        residual = axis_err - Jr_perp @ dq_pos
        dq_axis = null @ (Jr_hat.T @ (_inv3(Jr_hat @ Jr_hat.T + 0.05**2 * EYE3) @ residual))
        axis_norm = math.sqrt(float(dq_axis @ dq_axis))
        if axis_norm > 0.15:
            dq_axis *= 0.15 / axis_norm
        return dq_pos + dq_axis

    def solve(self, target, locked=None, target_axis=None, axis_index=0, orientation_weight=0.08):
        q = self._get_q().copy()
        locked = locked or {}
        active_indices = [i for i in range(len(self.jnt_ids)) if i not in locked]
        lo, hi = self.lo[active_indices], self.hi[active_indices]
        target = np.asarray(target, dtype=float)
        target_axis = None if target_axis is None else np.asarray(target_axis, dtype=float)
        if target_axis is not None:
            norm = float(np.linalg.norm(target_axis))
            if norm > 1e-9:
                target_axis = target_axis / norm
            else:
                target_axis = None
        for idx, value in locked.items():
            q[idx] = value
        prev_axis_err = float("inf")
        stalled = 0
        for it in range(self.max_iter):
            for idx, value in locked.items():
                q[idx] = value
            self._set_q(q)
            mujoco.mj_kinematics(self.m, self.d)
            # mj_jacSite reads cdof/subtree_com, which only mj_comPos updates.
            # Without it the Jacobian stays frozen at the seed pose.
            mujoco.mj_comPos(self.m, self.d)
            pos_err = target - self.d.site_xpos[self.ee_id]
            Jp, Jr = self._jacobian(active_indices)
            Jr_perp = None
            axis_err = None
            axis_err_norm = 0.0
            if target_axis is not None:
                site_xmat = self.d.site_xmat[self.ee_id].reshape(3, 3)
                current_axis = site_xmat[:, axis_index]
                axis_cross = _cross3(current_axis, target_axis)
                cross_norm = _norm3(axis_cross)
                axis_dot = min(1.0, max(-1.0, float(current_axis @ target_axis)))
                axis_angle = math.atan2(cross_norm, axis_dot)
                if cross_norm > 1e-9:
                    axis_err = axis_cross * (axis_angle / cross_norm)
                elif axis_dot < 0.0:
                    # Antiparallel axes have a zero cross product but a pi-rad
                    # error. Choose a stable perpendicular rotation direction.
                    reference = np.array([1.0, 0.0, 0.0])
                    if abs(float(np.dot(current_axis, reference))) > 0.9:
                        reference = np.array([0.0, 1.0, 0.0])
                    rotation_axis = np.cross(current_axis, reference)
                    rotation_axis /= max(float(np.linalg.norm(rotation_axis)), 1e-9)
                    axis_err = rotation_axis * axis_angle
                else:
                    axis_err = np.zeros(3, dtype=float)
                axis_err_norm = axis_angle
                # Spinning about the tool axis does not change where it points,
                # so only constrain the two perpendicular rotation components.
                Jr_perp = Jr - np.outer(current_axis, current_axis @ Jr)
            en = _norm3(pos_err)
            if en < self.tol and axis_err_norm < 0.03:
                break
            # When the requested axis is unreachable at this XYZ, stop once the
            # position is met and the tilt has stopped improving.
            if en < self.tol and prev_axis_err - axis_err_norm < 2e-4:
                stalled += 1
                if stalled >= 5:
                    break
            else:
                stalled = 0
            prev_axis_err = axis_err_norm
            q_active = q[active_indices]
            free = np.ones(len(active_indices))
            dq_active = self._priority_step(Jp, Jr_perp, pos_err, axis_err, free)
            # Clipping a joint that is pushed into its limit stalls the solve.
            # Pin such joints and let the remaining joints take up the motion.
            pinned = ((q_active <= lo + 1e-4) & (dq_active < 0.0)) | \
                     ((q_active >= hi - 1e-4) & (dq_active > 0.0))
            if pinned.any() and not pinned.all():
                free = (~pinned).astype(float)
                dq_active = self._priority_step(Jp, Jr_perp, pos_err, axis_err, free)
            dq_active = self.alpha * dq_active
            step_norm = float(np.linalg.norm(dq_active))
            if step_norm > 0.25:
                dq_active *= 0.25 / step_norm
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
        self.cv_points = load_cv_points()
        self.ik_pose_points = load_ik_pose_points()
        self.radius_limit_cache = {}
        self.plan_cache = {}
        self._hardware_seed_cache = None
        self._hardware_seed_cache_time = 0.0

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
    def _solve_best_ik(self, x, y, z, wrist_angle=None, orientation="free", exhaustive=True,
                       seed_q=None, return_all=False, enough=None):
        """Try IK from live hardware, current simulation, and neutral seeds.

        With return_all, every seed's result is returned as
        (q, err, iters, axis_err) sorted best-first, so callers can try
        alternative elbow/wrist branches instead of only the best one.
        With enough=N, return_all stops once N distinct (>0.2 rad apart)
        sub-millimetre, axis-aligned solutions have been found.
        """
        with self.lock:
            qpos_snapshot = self.d.qpos.copy()
            qvel_snapshot = self.d.qvel.copy()
            current_q = [float(self.d.ctrl[self.act_ids[name]]) for name in ARM_JOINTS]
        if seed_q is not None:
            current_q = [float(value) for value in seed_q]

        target = np.array([x, y, z], dtype=float)
        target_axis = orientation_axis_for(orientation, x, y)
        base_yaw = math.atan2(y, x) if abs(x) > 1e-9 or abs(y) > 1e-9 else 0.0
        seeds = ik_search_seeds(base_yaw, current_q) if exhaustive else [current_q]
        # Reuse nearby solutions from completed moves. These preserve useful
        # elbow branches for neighboring targets that are hard to solve from
        # the folded home pose.
        nearby = sorted(
            self.ik_pose_points,
            key=lambda point: float(np.linalg.norm(target - np.asarray(point["xyz_m"], dtype=float))),
        )
        nearby_count = 8 if exhaustive else 2
        for point in nearby[:nearby_count]:
            distance = float(np.linalg.norm(target - np.asarray(point["xyz_m"], dtype=float)))
            if distance <= 0.25:
                seeds.append(point["joints_rad"].copy())
        hardware_q = self._live_hardware_seed()
        if hardware_q is not None:
            # The physical arm may already be near the requested pose while the
            # MuJoCo simulation is at home. Seeding from its live joints avoids
            # losing reachable branches at tight workspace boundaries.
            seeds.insert(0, hardware_q)
        best = None
        best_score = None
        best_axis_err = float("inf")
        results = []
        good_branches = []
        scratch = mujoco.MjData(self.m)
        local_ik = DLSIK(self.m, scratch, EE_SITE, ARM_JOINTS)
        if not exhaustive:
            # Fast passes are only used to choose an approach height or follow
            # a descent path. Strict endpoint checks still use the full search.
            local_ik.max_iter = 80
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
                axis_err = math.atan2(
                    float(np.linalg.norm(np.cross(current_axis, target_axis))),
                    float(np.clip(np.dot(current_axis, target_axis), -1.0, 1.0)),
                )
            score = err + 0.08 * axis_err
            results.append((score, (q, err, iters, axis_err)))
            if best is None or score < best_score:
                best = (q, err, iters)
                best_score = score
                best_axis_err = axis_err
            # A sub-millimetre positional solution with the requested axis
            # already met is good enough; testing every remaining seed only
            # adds latency and can select a different elbow branch.
            if not return_all and best[1] <= 0.001 and best_axis_err <= 0.05:
                break
            if return_all and enough and err <= 0.001 and axis_err <= 0.05:
                if all(max(abs(a - b) for a, b in zip(q, other)) >= 0.2 for other in good_branches):
                    good_branches.append(q)
                if len(good_branches) >= enough:
                    break

        if return_all:
            return [result for _, result in sorted(results, key=lambda item: item[0])]
        return best

    def _solve_from_seed(self, xyz, seed_q, wrist_angle=None, orientation="free", max_iter=80):
        """Local IK from exactly one seed, for tracking a continuous path.

        Unlike preview_ik this never mixes in other seeds, so consecutive
        solutions stay on the same elbow/wrist branch.
        Returns (q, err, iters, axis_err).
        """
        scratch = mujoco.MjData(self.m)
        local_ik = DLSIK(self.m, scratch, EE_SITE, ARM_JOINTS)
        local_ik.max_iter = max_iter
        seed = [float(v) for v in seed_q]
        if wrist_angle is not None:
            seed[4] = wrist_angle
        local_ik._set_q(seed)
        mujoco.mj_kinematics(self.m, scratch)
        x, y, z = (float(v) for v in xyz)
        target_axis = orientation_axis_for(orientation, x, y)
        locked = {4: wrist_angle} if wrist_angle is not None else None
        q, err, iters = local_ik.solve(np.array([x, y, z]), locked=locked, target_axis=target_axis)
        return q, err, iters, self._tool_axis_error(q, orientation, x, y)

    def _fk_frame(self, q):
        """Return (tool position, tool rotation matrix, wrist_roll axis) for q."""
        scratch = mujoco.MjData(self.m)
        for k, ji in enumerate(self.ik.jnt_ids):
            scratch.qpos[self.m.joint(ji).qposadr[0]] = q[k]
        mujoco.mj_kinematics(self.m, scratch)
        pos = scratch.site_xpos[self.ee_id].copy()
        rot = scratch.site_xmat[self.ee_id].reshape(3, 3).copy()
        roll_axis = scratch.xaxis[self.jnt_ids["wrist_roll"]].copy()
        return pos, rot, roll_axis

    def _fk(self, q):
        """Return (tool position, tool pointing axis) for arm joints q."""
        pos, rot, _ = self._fk_frame(q)
        return pos, rot[:, 0].copy()

    def _jaw_yaw(self, q):
        """World yaw (radians) of the line the jaws close along.

        The moving jaw swings about the gripper hinge, so the jaws close along
        tool_axis x hinge_axis, which is the gripperframe z axis.
        """
        _, rot, _ = self._fk_frame(q)
        close = rot[:, 2]
        return math.atan2(float(close[1]), float(close[0]))

    def _jaw_aligned_rolls(self, x, y, z, orientation, jaw_yaw):
        """Wrist roll values that put the jaw closing line at jaw_yaw.

        Roll and base pan both turn the jaws about the vertical, so the roll
        depends on the target XY. Converge it with local solves, then return
        every equivalent roll (the line repeats every 180 deg) inside the
        joint range, so the planner can pick the one with the best path.
        """
        q = list(self._solve_best_ik(x, y, z, orientation=orientation, exhaustive=False)[0])
        roll = q[4]
        for _ in range(6):
            _, _, roll_axis = self._fk_frame(q)
            # Turning roll by +d turns the jaws by +d about the roll axis,
            # which points up (+Z) when the tool points down.
            sign = 1.0 if roll_axis[2] >= 0.0 else -1.0
            error = wrap_line_angle(self._jaw_yaw(q) - jaw_yaw)
            if abs(error) < math.radians(0.2):
                break
            roll -= sign * error
            # Same jaw line, shifted back inside the joint range.
            while roll > WRIST_ROLL_RANGE[1]:
                roll -= math.pi
            while roll < WRIST_ROLL_RANGE[0]:
                roll += math.pi
            q[4] = roll
            q = list(self._solve_from_seed((x, y, z), q, wrist_angle=roll,
                                           orientation=orientation)[0])
        rolls = []
        for k in range(-2, 3):
            candidate = roll + k * math.pi
            if WRIST_ROLL_RANGE[0] <= candidate <= WRIST_ROLL_RANGE[1]:
                rolls.append(candidate)
        # Smallest twist from the neutral wrist first.
        return sorted(rolls, key=abs)

    def _tool_axis_error(self, q, orientation, x, y):
        """Angle in radians between the tool axis and the requested direction."""
        target_axis = orientation_axis_for(orientation, x, y)
        if target_axis is None:
            return 0.0
        _, axis = self._fk(q)
        return math.atan2(
            float(np.linalg.norm(np.cross(axis, target_axis))),
            float(np.clip(np.dot(axis, target_axis), -1.0, 1.0)),
        )

    def _live_hardware_seed(self):
        """Read and briefly cache live arm joints as an additional IK seed.

        The hardware readings use the same radians and joint names as the
        recorded hardware FK points. A failed/offline bridge simply falls back
        to the normal simulation and neutral seeds.
        """
        now = time.monotonic()
        if now - self._hardware_seed_cache_time < 0.5:
            return None if self._hardware_seed_cache is None else self._hardware_seed_cache.copy()
        self._hardware_seed_cache_time = now
        self._hardware_seed_cache = None
        try:
            with urllib.request.urlopen(HW_BRIDGE_URL + "/joints", timeout=0.25) as response:
                payload = json.loads(response.read().decode("utf-8"))
            joints = payload.get("joints", {})
            seed = [float(joints[name]) for name in ARM_JOINTS]
            if not all(math.isfinite(value) for value in seed):
                return None
            limits = [self.m.joint(ji).range for ji in self.ik.jnt_ids]
            # Real arm limits/calibration can differ slightly from the MuJoCo
            # model. Accept small boundary overshoots and clip the seed when
            # solving; larger differences still indicate a bad reading.
            if any(value < lo - 0.15 or value > hi + 0.15 for value, (lo, hi) in zip(seed, limits)):
                return None
            seed = [min(hi, max(lo, value)) for value, (lo, hi) in zip(seed, limits)]
            self._hardware_seed_cache = seed
            return seed.copy()
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            return None

    def _remember_ik_pose(self, xyz, joints, error):
        """Persist accurate solved poses as seeds for nearby future targets."""
        if error > 0.015:
            return
        point = {
            "xyz_m": [float(v) for v in xyz],
            "joints_rad": [float(v) for v in joints],
        }
        with self.lock:
            for index, old in enumerate(self.ik_pose_points):
                if float(np.linalg.norm(
                    np.asarray(point["xyz_m"]) - np.asarray(old["xyz_m"])
                )) < 0.005:
                    self.ik_pose_points[index] = point
                    break
            else:
                self.ik_pose_points.append(point)
            self.ik_pose_points = self.ik_pose_points[-128:]
            try:
                save_ik_pose_points(self.ik_pose_points)
            except OSError as e:
                print(f"  Could not save learned IK pose: {e}")

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
        self._remember_ik_pose((x, y, z), q, err)

        print(f"  IK: current_ctrl={[f'{v:.3f}' for v in current_q]}, target={[f'{v:.3f}' for v in q]}" +
              (f", wrist_angle={wrist_angle:.3f}" if wrist_angle is not None else "") +
              (f", orientation={orientation}" if orientation != "free" else ""))
        self._start_motion(current_q, q, smooth, duration)
        return q, err, iters

    def move_to_joints(self, q, xyz, duration=2.0):
        """Move to a joint solution validated by a preflight, without re-solving.

        Re-running the seed search at execution time can pick a different
        elbow/wrist branch than the one that was checked, so the arm could
        swing through an unchecked arc near the table.
        Returns the solution's XYZ error against the target in metres.
        """
        q = [float(v) for v in q]
        with self.lock:
            current_q = [float(self.d.ctrl[self.act_ids[name]]) for name in ARM_JOINTS]
            self.d.qpos[self.tgt_ids[0]] = xyz[0]
            self.d.qpos[self.tgt_ids[1]] = xyz[1]
            self.d.qpos[self.tgt_ids[2]] = xyz[2]
        pos, _ = self._fk(q)
        err = float(np.linalg.norm(np.asarray(xyz, dtype=float) - pos))
        self.last_ik_err = err
        self.last_ik_iters = 0
        self._start_motion(current_q, q, True, duration)
        return err

    def _start_motion(self, current_q, q, smooth, duration):
        # Set up interpolation or instant move (brief lock)
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

    def preview_ik(self, x, y, z, wrist_angle=None, orientation="free", exhaustive=True, seed_q=None):
        """Solve IK against scratch data without moving the live simulation."""
        orientation = parse_orientation(orientation)
        return self._solve_best_ik(
            x, y, z, wrist_angle=wrist_angle, orientation=orientation,
            exhaustive=exhaustive, seed_q=seed_q
        )

    def build_coordinate_action(self, command):
        """Expand a Cartesian target and gripper state into a safe mini-pipeline."""
        if self.pipeline_running:
            raise ValueError("A pipeline is already running")

        x = float(command["x"])
        y = float(command["y"])
        z = float(command["z"])
        duration = float(command.get("duration", 7.0))
        # Pause after each arm move; smooth streaming needs little settling.
        settle_time = float(command.get("settle_time", 0.4))
        gripper_settle = float(command.get("gripper_settle", 0.8))
        max_err_mm = float(command.get("max_err_mm", 15.0))
        approach_max_err_mm = float(command.get("approach_max_err_mm", max_err_mm + 10.0))
        # Tool-axis tolerances for orientation=downwards/horizontal. Position is
        # always solved first; these bound how far the tool may tilt instead.
        max_axis_err_deg = float(command.get("max_axis_err_deg", 10.0))
        approach_max_axis_err_deg = float(command.get("approach_max_axis_err_deg", 25.0))
        min_approach_clearance = float(command.get("min_approach_clearance", 0.04))
        # Direction of the jaw closing line from robot +X, for downward grasps.
        jaw_yaw_deg = float(command.get("jaw_yaw_deg", DEFAULT_JAW_YAW_DEG))
        # The SO-101 gripper is asymmetric: one fixed jaw carrying the motor,
        # one moving jaw. The two rolls that align the jaw line (180 deg apart)
        # put the motor on opposite sides of the object, and switching between
        # them shifts where the object is gripped. Keep one side unless it
        # cannot reach: "base" = motor jaw toward the robot base.
        fixed_jaw_side = str(command.get("fixed_jaw_side", DEFAULT_FIXED_JAW_SIDE)).lower()
        if fixed_jaw_side not in ("base", "away", "auto"):
            raise ValueError("fixed_jaw_side must be base, away, or auto")
        orientation = parse_orientation(command.get("orientation", command.get("gripper_orientation", "free")))
        wrist_angle = command.get("wrist_angle", None)
        if wrist_angle is not None:
            wrist_angle = float(wrist_angle)
            if not (WRIST_ROLL_RANGE[0] <= wrist_angle <= WRIST_ROLL_RANGE[1]):
                raise ValueError(
                    f"wrist_angle must be between {WRIST_ROLL_RANGE[0]:.2f} and "
                    f"{WRIST_ROLL_RANGE[1]:.2f} radians"
                )

        values = [x, y, z, duration, settle_time, gripper_settle,
                  max_err_mm, approach_max_err_mm, max_axis_err_deg,
                  approach_max_axis_err_deg, min_approach_clearance, jaw_yaw_deg]
        if wrist_angle is not None:
            values.append(wrist_angle)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("All numeric fields must be finite")
        if not (X_RANGE[0] <= x <= X_RANGE[1] and Y_RANGE[0] <= y <= Y_RANGE[1] and Z_RANGE[0] <= z <= Z_RANGE[1]):
            raise ValueError("Target is outside the configured XYZ bounds")
        radius = math.hypot(x, y)
        min_radius = float(command.get("min_radius_m", MIN_TARGET_RADIUS))
        if not (math.isfinite(min_radius) and 0.05 <= min_radius <= 0.25):
            raise ValueError("min_radius_m must be between 0.05 and 0.25")
        if radius < min_radius:
            raise ValueError(
                f"Target is too close to the base: radial distance {radius:.3f}m is below "
                f"{min_radius:.3f}m. Increase X or Y."
            )
        if not (0.2 <= duration <= 10.0 and 0.0 <= settle_time <= 5.0 and 0.0 <= gripper_settle <= 5.0):
            raise ValueError("Timing values are outside the allowed range")
        if not (1.0 <= max_err_mm <= 100.0):
            raise ValueError("max_err_mm must be between 1 and 100")
        if not (max_err_mm <= approach_max_err_mm <= 100.0):
            raise ValueError("approach_max_err_mm must be between max_err_mm and 100")
        if not (0.5 <= max_axis_err_deg <= approach_max_axis_err_deg <= 90.0):
            raise ValueError(
                "Need 0.5 <= max_axis_err_deg <= approach_max_axis_err_deg <= 90"
            )
        if not (0.0 <= min_approach_clearance <= 0.3):
            raise ValueError("min_approach_clearance must be between 0 and 0.3 m")

        requested_approach_z = command.get("approach_z", None)
        if requested_approach_z is None:
            # Aim high for a safe approach, but accept the highest height the
            # arm can really hold above this target with the tool aligned.
            # Close-in downward grasps run out of wrist flex well below 18 cm,
            # which is what made fixed 18-30 cm approach heights fail.
            desired_approach_z = min(max(z + 0.15, 0.18), Z_RANGE[1])
            min_clearance = min_approach_clearance
        else:
            desired_approach_z = float(requested_approach_z)
            if not math.isfinite(desired_approach_z):
                raise ValueError("approach_z must be finite")
            if not (Z_RANGE[0] <= desired_approach_z <= Z_RANGE[1]):
                raise ValueError("Approach height is outside the configured Z bounds")
            desired_approach_z = max(desired_approach_z, z)
            # An explicit approach height must be reached, not shortened.
            min_clearance = desired_approach_z - z

        pre_grip_value = parse_gripper_value(command.get("pre_gripper", "hold"), "pre_gripper")
        post_gripper = command.get("post_gripper", command.get("gripper", command.get("gripper_action", "hold")))
        post_grip_value = parse_gripper_value(command.get("gripper_value", post_gripper), "post_gripper")

        # Choose one wrist roll for every pose of this action. It is set on the
        # approach move, before reaching the pick position, and held through
        # descent, grasp and retreat. Downward grasps turn the jaws parallel
        # to robot X (or jaw_yaw_deg); an explicit wrist_angle overrides it.
        jaw_aligned = wrist_angle is None and orientation == "downwards"
        if wrist_angle is not None:
            roll_mode = ("fixed", round(wrist_angle, 4))
        elif jaw_aligned:
            roll_mode = ("jaw_yaw", round(jaw_yaw_deg, 2), fixed_jaw_side)
        else:
            roll_mode = ("snapped",)
        # Clients preview a target and then run it with the same fields; reuse
        # that plan instead of repeating the roll search.
        cache_key = (round(x, 4), round(y, 4), round(z, 4), round(desired_approach_z, 4),
                     orientation, max_err_mm, approach_max_err_mm, max_axis_err_deg,
                     approach_max_axis_err_deg, roll_mode)
        cached = self.plan_cache.get(cache_key)
        jaw_side_note = None
        if cached is not None and time.monotonic() - cached[0] < PLAN_CACHE_SECONDS:
            plan, wrist_angle, jaw_side_note = cached[1], cached[2], cached[3]
            roll_options = []
        else:
            plan = None
            if wrist_angle is not None:
                roll_options = [wrist_angle]
            elif jaw_aligned:
                roll_options = self._jaw_aligned_rolls(x, y, z, orientation, math.radians(jaw_yaw_deg))
            else:
                roll_options = SNAPPED_WRIST_ROLLS
        roll_plans = []
        for roll in roll_options:
            for _ in range(3 if jaw_aligned else 1):
                candidate = self._plan_vertical_access(
                    x, y, z, desired_approach_z, roll, orientation,
                    max_err_mm / 1000.0, approach_max_err_mm / 1000.0,
                    math.radians(max_axis_err_deg), math.radians(approach_max_axis_err_deg),
                )
                if not jaw_aligned or candidate["grasp"] is None:
                    break
                # The full grasp search may settle on a slightly different
                # base pan than the roll estimate assumed; correct and replan.
                grasp_q = candidate["grasp"][0]
                jaw_err = wrap_line_angle(self._jaw_yaw(grasp_q) - math.radians(jaw_yaw_deg))
                if abs(jaw_err) < math.radians(0.5):
                    break
                sign = 1.0 if self._fk_frame(grasp_q)[2][2] >= 0.0 else -1.0
                corrected = roll - sign * jaw_err
                if not (WRIST_ROLL_RANGE[0] <= corrected <= WRIST_ROLL_RANGE[1]):
                    break
                roll = corrected
            roll_plans.append((candidate, roll))
            if plan is None or self._plan_rank(candidate) > self._plan_rank(plan):
                plan, wrist_angle = candidate, roll
            # With a side preference both 180-degree options must be seen.
            if (not (jaw_aligned and fixed_jaw_side != "auto") and plan["grasp"] is not None
                    and plan["chain"][-1][0] >= desired_approach_z - 1e-6):
                break  # Full clearance; a later roll cannot do better.
        if roll_plans and jaw_aligned:
            plan, wrist_angle, jaw_side_note = self._choose_jaw_side(
                roll_plans, x, y, z, fixed_jaw_side, min_clearance)
        if roll_options:
            now = time.monotonic()
            self.plan_cache = {key: value for key, value in self.plan_cache.items()
                               if now - value[0] < PLAN_CACHE_SECONDS}
            self.plan_cache[cache_key] = (now, plan, wrist_angle, jaw_side_note)

        preflight = []
        failed_checks = []
        if plan["grasp"] is None:
            _, err, iters, axis_err = plan["best_grasp"]
            preflight.append({"label": "Move to coordinate", "err_mm": round(err * 1000.0, 2),
                              "iters": iters, "tilt_deg": round(math.degrees(axis_err), 1)})
            if err * 1000.0 > max_err_mm:
                failed_checks.append(
                    f"Move to coordinate IK error {err * 1000.0:.1f}mm exceeds {max_err_mm:.1f}mm"
                )
            else:
                failed_checks.append(
                    f"Move to coordinate tool tilt {math.degrees(axis_err):.1f}deg exceeds "
                    f"{max_axis_err_deg:.1f}deg for orientation={orientation}"
                )
        else:
            grasp_q, grasp_err, grasp_iters, grasp_axis = plan["grasp"]
            chain = plan["chain"]
            approach_z, approach_q, approach_err, approach_axis = chain[-1]
            clearance = approach_z - z
            preflight.extend([
                {"label": f"Approach target (wrist roll {math.degrees(wrist_angle):+.0f}deg)",
                 "err_mm": round(approach_err * 1000.0, 2),
                 "iters": None, "tilt_deg": round(math.degrees(approach_axis), 1),
                 "z": round(approach_z, 4), "wrist_roll_deg": round(math.degrees(wrist_angle), 1)},
                {"label": "Move to coordinate", "err_mm": round(grasp_err * 1000.0, 2),
                 "iters": grasp_iters, "tilt_deg": round(math.degrees(grasp_axis), 1),
                 "z": round(z, 4)},
                {"label": "Vertical descent path (worst sample)",
                 "err_mm": round(max(item[2] for item in chain) * 1000.0, 2),
                 "iters": None,
                 "tilt_deg": round(math.degrees(max(item[3] for item in chain)), 1)},
            ])
            if jaw_side_note:
                preflight.append({"label": jaw_side_note, "err_mm": 0.0, "iters": None})
            if jaw_aligned:
                jaw_err = math.degrees(wrap_line_angle(
                    self._jaw_yaw(grasp_q) - math.radians(jaw_yaw_deg)))
                preflight.append({"label": f"Jaw line vs robot X (target {jaw_yaw_deg:+.0f}deg)",
                                  "err_mm": 0.0, "iters": None,
                                  "jaw_yaw_err_deg": round(jaw_err, 2)})
                if abs(jaw_err) > MAX_JAW_YAW_ERR_DEG:
                    failed_checks.append(
                        f"Jaw line is {jaw_err:+.1f}deg off {jaw_yaw_deg:+.0f}deg from robot X "
                        f"at the pick position (limit {MAX_JAW_YAW_ERR_DEG:.0f}deg)"
                    )
            if clearance + 1e-6 < min_clearance:
                failed_checks.append(
                    f"Approach clearance only {clearance * 100.0:.1f}cm above the target "
                    f"(need {min_clearance * 100.0:.1f}cm) within {approach_max_err_mm:.0f}mm / "
                    f"{approach_max_axis_err_deg:.0f}deg tool tilt; the arm cannot hold "
                    f"orientation={orientation} higher above this XY"
                )

        if failed_checks:
            error = "; ".join(failed_checks)
            # Avoid an expensive 25-radius workspace sweep just to decorate a
            # failed pick/place response. It repeated IK solves and could add
            # seconds to every rejected target.
            hint = ("Check the recorded CV-to-IK points and target height/orientation, "
                    "or relax approach_max_axis_err_deg / min_approach_clearance.")
            raise ValueError(f"{error}. {hint}")

        # Keep accurate preview solutions as seeds too. This makes the next
        # target check less dependent on whether the physical arm was left at
        # home, stopped mid-motion, or near the previous pick/place pose.
        self._remember_ik_pose((x, y, approach_z), approach_q, approach_err)
        self._remember_ik_pose((x, y, z), grasp_q, grasp_err)

        waypoints = self._descent_waypoints(x, y, chain)
        descent_total = max(approach_z - z, 1e-6)

        def move_step(label, step_z, q, step_duration, step_settle):
            step = {
                "type": "move_ik",
                "label": label,
                "x": x,
                "y": y,
                "z": round(float(step_z), 5),
                "orientation": orientation,
                "duration": round(step_duration, 3),
                "settle_time": step_settle,
                # Exact preflight joints: execution must not re-solve onto a
                # different branch than the one checked above.
                "joints": dict(zip(ARM_JOINTS, [float(v) for v in q])),
            }
            if wrist_angle is not None:
                step["wrist_angle"] = wrist_angle
            return step

        def path_moves(points, final_label, final_settle):
            """Moves through (z, q) points after the first, timed by height."""
            moves = []
            prev_z = points[0][0]
            for index, (step_z, q) in enumerate(points[1:], start=1):
                share = abs(prev_z - step_z) / descent_total
                is_last = index == len(points) - 1
                moves.append(move_step(
                    final_label if is_last else f"{final_label} (waypoint {index})",
                    step_z, q, max(0.5, duration * share),
                    final_settle if is_last else 0.0,
                ))
                if not is_last:
                    # Flow through this waypoint: the pipeline sends the run
                    # as one continuous path instead of stopping here.
                    moves[-1]["blend_next"] = True
                prev_z = step_z
            return moves

        has_descent = approach_z - z >= 1e-6
        down_path = [(approach_z, approach_q)] + \
            [(item[0], item[1]) for item in waypoints] + [(z, grasp_q)]

        steps = []
        if pre_grip_value is not None:
            steps.append({
                "type": "gripper",
                "label": "Open gripper before move" if pre_grip_value < 0.5 else "Close gripper before move",
                "value": pre_grip_value,
                "settle_time": gripper_settle,
            })
        steps.append(move_step("Approach target", approach_z, approach_q, duration, settle_time))
        if has_descent:
            steps.extend(path_moves(down_path, "Move to coordinate", settle_time))
        if post_grip_value is not None:
            steps.append({
                "type": "gripper",
                "label": "Close gripper after reaching target" if post_grip_value >= 0.5 else "Open gripper after reaching target",
                "value": post_grip_value,
                "settle_time": gripper_settle,
                # A pick's close is checked: stop if the jaws met nothing.
                "verify_grip": post_grip_value >= 0.5 and command.get("verify_grip", True),
            })
        if command.get("retreat", True) and has_descent:
            steps.extend(path_moves(list(reversed(down_path)), "Retreat above target", settle_time))

        if waypoints:
            preflight.append({"label": f"Descent waypoints inserted: {len(waypoints)}",
                              "err_mm": 0.0, "iters": None})
        contact_mm, parts = self._self_contact_mm(steps)
        if contact_mm < -SELF_CONTACT_WARN_MM:
            preflight.append({
                "label": (f"WARNING: model predicts {parts} contact ({-contact_mm:.0f} mm); "
                          "the real arm may clear it - watch the first run here"),
                "err_mm": 0.0, "iters": None, "self_contact_mm": round(contact_mm, 1)})
        return steps, preflight

    def _self_contact_mm(self, steps):
        """Deepest arm self-penetration over the planned poses (mm, <0 = overlap)."""
        floor = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        deepest, parts = 0.0, ""
        data = mujoco.MjData(self.m)
        for step in steps:
            if step.get("type") != "move_ik" or "joints" not in step:
                continue
            for name in ARM_JOINTS:
                data.qpos[self.m.joint(self.jnt_ids[name]).qposadr[0]] = step["joints"][name]
            mujoco.mj_forward(self.m, data)
            for contact in data.contact[:data.ncon]:
                if floor in (contact.geom1, contact.geom2) or contact.dist >= deepest:
                    continue
                deepest = float(contact.dist)
                parts = "/".join(self.m.body(self.m.geom_bodyid[g]).name
                                 for g in (contact.geom1, contact.geom2))
        return deepest * 1000.0, parts

    def _fixed_jaw_toward_base(self, q, x, y):
        """Whether the motor (fixed) jaw sits on the robot-base side of the
        target, and how clearly (|cos| of jaw line vs base direction)."""
        _, rot, _ = self._fk_frame(q)
        # gripperframe z points from the fixed jaw toward the moving jaw.
        opening = rot[:2, 2]
        radius = math.hypot(x, y)
        to_base = np.array([-x, -y]) / max(radius, 1e-9)
        cos = float(opening @ to_base) / max(float(np.linalg.norm(opening)), 1e-9)
        return cos < 0.0, abs(cos)

    def _choose_jaw_side(self, roll_plans, x, y, z, side, min_clearance):
        """Pick between the jaw-aligned rolls, keeping one motor side.

        Returns (plan, roll, note). Falls back to the other side only when the
        preferred one cannot reach the target with the required clearance.
        """
        def feasible(item):
            p = item[0]
            return p["grasp"] is not None and p["chain"][-1][0] - z + 1e-6 >= min_clearance

        def describe(item):
            toward, _ = self._fixed_jaw_toward_base(item[0]["grasp"][0], x, y)
            return "motor jaw toward base" if toward else "motor jaw away from base"

        best_overall = max(roll_plans, key=lambda item: self._plan_rank(item[0]))
        if side == "auto":
            chosen = best_overall
            note = describe(chosen) if chosen[0]["grasp"] is not None else None
            return chosen[0], chosen[1], note and f"Gripper side: {note} (auto)"
        preferred, other = [], []
        for item in roll_plans:
            if item[0]["grasp"] is None:
                continue
            toward, clarity = self._fixed_jaw_toward_base(item[0]["grasp"][0], x, y)
            # A jaw line almost perpendicular to the base direction has no
            # meaningful "toward base" side; treat both as preferred.
            match = clarity < 0.1 or (toward == (side == "base"))
            (preferred if match else other).append(item)
        wanted = "toward base" if side == "base" else "away from base"
        for pool, note in ((preferred, f"Gripper side: motor jaw {wanted}"),
                           (other, f"Gripper side FLIPPED: motor jaw {wanted} cannot reach here")):
            reachable = [item for item in pool if feasible(item)]
            if reachable:
                chosen = max(reachable, key=lambda item: self._plan_rank(item[0]))
                return chosen[0], chosen[1], note
        return best_overall[0], best_overall[1], None

    @staticmethod
    def _plan_rank(plan):
        """Order plans: has a grasp, then most clearance, then least grasp tilt."""
        if plan["grasp"] is None:
            _, err, _, axis_err = plan["best_grasp"]
            return (0, 0.0, -err - 0.08 * axis_err)
        return (1, round(plan["chain"][-1][0], 4), -plan["grasp"][3])

    def _plan_vertical_access(self, x, y, z, top_z, wrist_angle, orientation,
                              max_err, approach_max_err, max_axis, approach_max_axis):
        """Find a grasp pose and a continuous vertical path above it.

        The grasp is solved first because it is the pose that must be exact.
        The approach is then found by climbing from that grasp in 1 cm steps,
        seeding each step from the previous one, so approach, descent and
        grasp all stay on one arm branch (no elbow flip next to the table).
        Several distinct grasp branches are tried; the one with the most
        clearance wins. Errors are metres, axis limits radians.

        Returns {"best_grasp", "grasp", "chain"} where chain is a list of
        (z, q, err, axis_err) from the grasp (index 0) up to the approach.
        """
        results = self._solve_best_ik(
            x, y, z, wrist_angle=wrist_angle, orientation=orientation,
            exhaustive=True, return_all=True, enough=3,
        )
        grasps = []
        for q, err, iters, axis_err in results:
            if err > max_err or axis_err > max_axis:
                continue
            if any(max(abs(a - b) for a, b in zip(q, other[0])) < 0.2 for other in grasps):
                continue  # Same branch as a grasp already kept.
            grasps.append((q, err, iters, axis_err))
            if len(grasps) >= 4:
                break

        plan = {"best_grasp": results[0], "grasp": None, "chain": []}
        climb_steps = max(1, int(math.ceil((top_z - z) / 0.01)))
        for grasp in grasps:
            chain = [(z, grasp[0], grasp[1], grasp[3])]
            prev_q = grasp[0]
            # Hold the grasp's roll on the vertical path. Left free, the axis
            # task spins roll to its limit once wrist_flex saturates, for
            # almost no tilt gain, twisting the jaws while descending.
            climb_roll = wrist_angle if wrist_angle is not None else grasp[0][4]
            for step_index in range(1, climb_steps + 1):
                step_z = z + (top_z - z) * step_index / climb_steps
                q, err, _, axis_err = self._solve_from_seed(
                    (x, y, step_z), prev_q, wrist_angle=climb_roll, orientation=orientation
                )
                # Branch flips show up in pan/shoulder/elbow/wrist_flex. Roll
                # legitimately swings once wrist_flex saturates and it is used
                # to hold the tilt, so it is not part of this check.
                jump = max(abs(a - b) for a, b in zip(q[:4], prev_q[:4]))
                if err > approach_max_err or axis_err > approach_max_axis or jump > 0.35:
                    break
                chain.append((step_z, q, err, axis_err))
                prev_q = q
            if plan["grasp"] is None or chain[-1][0] > plan["chain"][-1][0] + 1e-6:
                plan["grasp"], plan["chain"] = grasp, chain
            if chain[-1][0] >= top_z - 1e-6:
                break
        return plan

    def _descent_waypoints(self, x, y, chain, max_dev=0.012):
        """Pick chain samples so joint-space moves stay close to the vertical.

        The pipeline interpolates joints between poses, which bows the tool
        away from the straight line over the target. Split a segment at its
        middle chain sample while the bow (sideways, or dipping below the
        lower end) exceeds max_dev metres. Returned top-to-bottom, excluding
        the approach and grasp poses themselves.
        """
        def deviation(i, j):
            qa = np.asarray(chain[i][1], dtype=float)
            qb = np.asarray(chain[j][1], dtype=float)
            low_z = min(chain[i][0], chain[j][0])
            worst = 0.0
            for t in (0.25, 0.5, 0.75):
                pos, _ = self._fk((1.0 - t) * qa + t * qb)
                worst = max(worst, math.hypot(pos[0] - x, pos[1] - y), low_z - pos[2])
            return worst

        def split(i, j):
            if j - i <= 1 or deviation(i, j) <= max_dev:
                return []
            k = (i + j) // 2
            return split(i, k) + [k] + split(k, j)

        indices = split(0, len(chain) - 1)
        return [chain[k] for k in sorted(indices, reverse=True)]

    def radius_limits_for_z(self, z, wrist_angle=None, orientation="free", max_err_mm=15.0,
                            max_axis_err_deg=10.0):
        z = float(z)
        max_err_mm = float(max_err_mm)
        max_axis_err_deg = float(max_axis_err_deg)
        orientation = parse_orientation(orientation)
        if wrist_angle is not None:
            wrist_angle = float(wrist_angle)
            if not (WRIST_ROLL_RANGE[0] <= wrist_angle <= WRIST_ROLL_RANGE[1]):
                raise ValueError(
                    f"wrist_angle must be between {WRIST_ROLL_RANGE[0]:.2f} and "
                    f"{WRIST_ROLL_RANGE[1]:.2f} radians"
                )
        values = [z, max_err_mm, max_axis_err_deg]
        if wrist_angle is not None:
            values.append(wrist_angle)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("All numeric fields must be finite")
        if not (Z_RANGE[0] <= z <= Z_RANGE[1]):
            raise ValueError("Z is outside the configured bounds")
        if not (1.0 <= max_err_mm <= 100.0):
            raise ValueError("max_err_mm must be between 1 and 100")

        cache_key = (round(z, 3), None if wrist_angle is None else round(wrist_angle, 2), orientation,
                     round(max_err_mm, 1), round(max_axis_err_deg, 1))
        if cache_key in self.radius_limit_cache:
            return self.radius_limit_cache[cache_key].copy()

        reachable = []
        for radius in np.linspace(MIN_TARGET_RADIUS, 0.6, 25):
            q, err, _ = self.preview_ik(
                float(radius), 0.0, z, wrist_angle=wrist_angle,
                orientation=orientation, exhaustive=False
            )
            # Position is solved first, so an unreachable orientation shows
            # up as tool tilt rather than XYZ error.
            tilt = self._tool_axis_error(q, orientation, float(radius), 0.0)
            if err * 1000.0 <= max_err_mm and math.degrees(tilt) <= max_axis_err_deg:
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
            # Keep the busy flag set until the pipeline worker catches the
            # stop request and exits. Otherwise /home or a new action can
            # start while the cancelled worker is still unwinding.
            self.pipeline_log.append("STOP requested")

    def current_pose(self):
        with self.lock:
            mujoco.mj_forward(self.m, self.d)
            return {
                name: float(self.d.qpos[self.m.joint(self.jnt_ids[name]).qposadr[0]])
                for name in ALL_ACTS
            }

    def record_cv_point(self, name, board_xy_cm, joints=None):
        """Save a board coordinate paired with current arm-tip XYZ in world metres.

        If joint angles are provided (the UI sends these from the hardware
        bridge), calculate the end-effector position with MuJoCo forward
        kinematics. Otherwise capture the current simulated arm state.
        """
        name = str(name).strip()
        if not name or len(name) > 48:
            raise ValueError("Point name must contain 1 to 48 characters")
        if not isinstance(board_xy_cm, (list, tuple)) or len(board_xy_cm) != 2:
            raise ValueError("board_xy_cm must be [x, y]")
        board_xy_cm = [float(v) for v in board_xy_cm]
        if not all(math.isfinite(v) for v in board_xy_cm):
            raise ValueError("Board coordinates must be finite")

        if joints is None:
            with self.lock:
                mujoco.mj_forward(self.m, self.d)
                xyz = self.d.site_xpos[self.ee_id].copy().tolist()
            source = "sim"
        else:
            if not isinstance(joints, dict):
                raise ValueError("joints must be an object of joint names and radians")
            values = {}
            for joint in ARM_JOINTS:
                if joint not in joints:
                    raise ValueError(f"Missing hardware joint: {joint}")
                value = float(joints[joint])
                if not math.isfinite(value):
                    raise ValueError(f"Invalid hardware joint value for {joint}")
                values[joint] = value
            data = mujoco.MjData(self.m)
            mujoco.mj_resetData(self.m, data)
            for joint, value in values.items():
                qpos = self.m.joint(self.jnt_ids[joint]).qposadr[0]
                data.qpos[qpos] = value
            mujoco.mj_forward(self.m, data)
            xyz = data.site_xpos[self.ee_id].copy().tolist()
            source = "hardware_joint_fk"

        point = {
            "name": name,
            "board_xy_cm": board_xy_cm,
            "robot_xyz_m": [float(v) for v in xyz],
            "source": source,
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        self.cv_points[name] = point
        save_cv_points(self.cv_points)
        return point

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
    def _hardware_grip_result(self, wait_s=2.0):
        """Bridge's close-until-contact result, waiting while it is closing."""
        deadline = time.time() + wait_s
        while True:
            status = hw_get("/grip_status")
            if status is None:
                return None
            grip = status.get("grip", {})
            if grip.get("state") != "closing" or time.time() > deadline:
                return grip
            self._sleep_interruptible(0.1)

    def _sleep_interruptible(self, duration, interval=0.05):
        end_time = time.time() + max(0.0, float(duration))
        while time.time() < end_time:
            if self.pipeline_stop_requested:
                raise RuntimeError("Stopped by kill switch")
            time.sleep(min(interval, end_time - time.time()))

    @staticmethod
    def _merge_blended_moves(steps):
        """Join runs of joint moves marked blend_next into one path step."""
        merged, run = [], []
        for step in steps:
            joint_move = step.get("type", "move_ik") == "move_ik" and "joints" in step
            if joint_move and (run or step.get("blend_next")):
                run.append(step)
                if not step.get("blend_next"):
                    merged.append({"type": "move_path", "label": step.get("label", "Path"),
                                   "parts": run})
                    run = []
                continue
            merged.extend(run)
            run = []
            merged.append(step)
        merged.extend(run)
        return merged

    def run_pipeline(self, steps):
        if self.pipeline_running:
            raise ValueError("A pipeline is already running")
        steps = self._merge_blended_moves(steps)
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
                        duration = step.get("duration", 7.0)
                        settle_time = step.get("settle_time", 0.5)
                        # Get optional wrist angle for gripper orientation control
                        wrist_angle = step.get("wrist_angle", None)
                        orientation = step.get("orientation", "free")

                        # Start the motion. Coordinate actions carry the
                        # joints their preflight validated; plain pipeline
                        # steps are still solved here.
                        if "joints" in step:
                            q = [float(step["joints"][name]) for name in ARM_JOINTS]
                            err, iters = self.move_to_joints(q, (x, y, z), duration=duration), 0
                        else:
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
                    elif stype == "move_path":
                        # One continuous hardware motion through all parts.
                        parts = step["parts"]
                        mirror_hw("/move_path", {
                            "waypoints": [{n: float(p_["joints"][n]) for n in ARM_JOINTS}
                                          for p_ in parts],
                            "durations": [float(p_.get("duration", 1.0)) for p_ in parts],
                        })
                        for part in parts:
                            if self.pipeline_stop_requested:
                                raise RuntimeError("Stopped by kill switch")
                            q = [float(part["joints"][n]) for n in ARM_JOINTS]
                            err = self.move_to_joints(q, (part["x"], part["y"], part["z"]),
                                                      duration=part.get("duration", 1.0))
                            self._sleep_interruptible(part.get("duration", 1.0))
                            self.pipeline_log.append(f"  ✓ {part.get('label', 'waypoint')} "
                                                     f"err={err*1000:.1f}mm (no stop)")
                        self._sleep_interruptible(parts[-1].get("settle_time", 0.5))
                    elif stype == "gripper":
                        mirror_hw("/set_motion_time", {"time_ms": int(step.get("settle_time", 0.8) * 1000)})
                        self.set_gripper(step.get("value", 0))
                        mirror_hw("/gripper", {
                            "value": step.get("value", 0)
                        })
                        self._sleep_interruptible(step.get("settle_time", 0.8))
                        # Now inverted: value 0 = OPEN, value 1 = CLOSE
                        self.pipeline_log.append(f"  ✓ Gripper={'OPEN' if step.get('value',0)<0.5 else 'CLOSE'}")
                        if step.get("verify_grip"):
                            grip = self._hardware_grip_result()
                            if grip is None:
                                self.pipeline_log.append("  (grip not checked: hardware bridge offline)")
                            elif grip.get("state") == "empty":
                                raise RuntimeError(
                                    "Nothing was gripped: the gripper closed fully without meeting "
                                    "the object. Check the pick position (nudge) and object size.")
                            else:
                                self.pipeline_log.append(
                                    f"  ✓ Grip {grip.get('state')}: contact at "
                                    f"{grip.get('contact_angle')} rad, load {grip.get('load_pct')} %")
                    elif stype == "home":
                        duration = step.get("duration", 7.0)
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
        elif self.path == "/cv_points":
            self.send_json({"points": SIM.cv_points, "file": CV_POINTS_FILE})
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
                    max_axis_err_deg=body.get("max_axis_err_deg", 10.0),
                )
            except (KeyError, TypeError, ValueError) as e:
                self.send_json({"ok": False, "error": str(e)}, 400)
                return
            self.send_json({"ok": True, **limits})

        elif p == "/home":
            try:
                SIM.run_pipeline([{"type": "home", "label": "Home pose", "duration": 7.0}])
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

        elif p == "/cv_points":
            try:
                point = SIM.record_cv_point(
                    body["name"], body["board_xy_cm"], body.get("joints"))
            except (KeyError, TypeError, ValueError, OSError) as e:
                self.send_json({"ok": False, "error": str(e)}, 400)
                return
            self.send_json({"ok": True, "point": point, "points": SIM.cv_points,
                            "file": CV_POINTS_FILE})

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
