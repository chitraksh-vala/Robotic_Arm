"""
SO-ARM101 Hardware Bridge
=========================
Connects the IK simulation server to the real SO-ARM101 via lerobot/Feetech STS3215 protocol.

Usage:
  python3 hw_bridge.py --port /dev/ttyUSB0 [--baud 1000000]
  Windows: python hw_bridge.py --port COM4

Then in the dashboard, set HW URL to http://127.0.0.1:7843 and click Connect HW.

Every IK command from the dashboard is mirrored to the real arm.
The real arm's joint positions are also read back and displayed.

HTTP Endpoints:
  POST /move               { joints: {name: rad, ...} }
  POST /move_path          { waypoints: [{name: rad, ...}, ...], durations: [s, ...] }
                           - one continuous motion through all waypoints (no stops)
  POST /gripper            { value: 0..1 }
  POST /home
  POST /stop
  POST /set_motion_time    { time_ms: 0..10000 }  - Time to reach position (higher = slower, default: 1500ms)
  POST /set_speed          { speed: 0..1000 }     - Legacy, converts to time_ms
  POST /set_acceleration   { acceleration: 0..254 } - For Feetech fallback only
  GET  /joints             → current joint positions
  GET  /health             → per-servo load %, current A, voltage V, temperature C
  GET  /grip_status        → closing / holding / empty, contact angle and load
  GET  /motion_params      → current motion settings
  GET  /ping               → connection status

Motion Control for STS3215:
  Trajectories are streamed at TRAJECTORY_RATE_HZ as plain position goals; the
  servos' own time/speed profiles are switched off (Goal_Time = Goal_Velocity
  = 0) so each update does not restart a separate servo-side move, which made
  the arm pulse and vibrate. Single moves use a quintic (rest to rest); paths
  use a shape-preserving cubic through the waypoints (no overshoot, no stops).
  On connect the servos get LeRobot's SO-101 tuning (P 16, I 0, D 32); the
  factory P of 32 makes the arm shaky. Motion time default: 3500 ms.
"""
import argparse, json, math, time, threading, sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

BASE_DIR = Path(__file__).parent
HOME_POSE_FILE = BASE_DIR / "home_pose.json"
GRIPPER_POSITIONS_FILE = BASE_DIR / "gripper_positions.json"
JOINT_NAMES = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
PORT = 7843
# Trajectory streaming rate. 20 Hz with a 120 ms servo-side Goal_Time made the
# servo re-plan a short move on every update; denser plain position goals let
# the servo's position loop follow the smooth trajectory directly.
TRAJECTORY_RATE_HZ = 50.0
# LeRobot's SO-101 follower tuning ("lower P to avoid shakiness"; default 32).
SERVO_P_COEFFICIENT = 16
SERVO_I_COEFFICIENT = 0
SERVO_D_COEFFICIENT = 32
SERVO_ACCELERATION = 254
# LeRobot's SO-101 gripper protection. Closing on an object stalls the
# gripper servo; unlimited, its stall current sags the shared supply and the
# whole arm jitters while holding. 500 = 50 % torque, 250 x 6.5 mA ~ 1.6 A.
GRIPPER_TORQUE_LIMIT = 500
GRIPPER_PROTECTION_CURRENT = 250
GRIPPER_OVERLOAD_TORQUE = 25
# Close-until-contact gripping. Closing ramps the gripper at GRIP_SPEED and
# watches its load and how far it lags the command; contact (either over its
# threshold for GRIP_CONTACT_TICKS ticks in a row) freezes the command a small
# squeeze past the contact angle instead of stalling against the object.
GRIP_SPEED_RAD_S = 1.2
GRIP_LOAD_THRESHOLD = 300          # 0.1 % units: 30 % of max load
GRIP_LAG_THRESHOLD_RAD = 0.15      # blocked: actual 8.6 deg behind command
GRIP_CONTACT_TICKS = 3             # 60 ms at 50 Hz: ignore one noisy reading
GRIP_SQUEEZE_RAD = math.radians(5.0)
# Contact this close to the saved fully-closed angle means the jaws met each
# other: nothing (or something thinner than ~5 mm) was gripped.
GRIP_EMPTY_MARGIN_RAD = 0.10
GRIP_TIMEOUT_S = 3.0
# Close slightly past the MuJoCo model's nominal lower limit so the real gripper
# clamps fully; tune this back toward -0.17453 if the servo strains.
GRIPPER_CLOSED_ANGLE = -0.55
GRIPPER_FULL_OPEN_ANGLE = 1.7453
GRIPPER_OPEN_FRACTION = 0.35
DEFAULT_GRIPPER_POSITIONS = {
    "open": GRIPPER_CLOSED_ANGLE + GRIPPER_OPEN_FRACTION * (GRIPPER_FULL_OPEN_ANGLE - GRIPPER_CLOSED_ANGLE),
    "close": GRIPPER_CLOSED_ANGLE,
}

# Folded rest pose used by /home. Angles are radians.
# Gripper matches the sim convention: closed is the physical closed limit.
HOME_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 1.45,
    "elbow_flex": -1.45,
    "wrist_flex": -1.20,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_CLOSED_ANGLE,
}

def gripper_value_to_angle(value):
    """Map UI/API gripper value where 0=part-open and 1=fully closed to joint angle."""
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
    for name in JOINT_NAMES:
        if name not in pose:
            raise ValueError(f"Missing home pose joint: {name}")
        value = float(pose[name])
        if not math.isfinite(value):
            raise ValueError(f"Invalid home pose value for {name}")
        validated[name] = value
    return validated

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
        "joint_order": JOINT_NAMES,
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    with open(HOME_POSE_FILE, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    return pose

# ── Try to import lerobot / feetech driver ──────────────────────────────────
HAS_LEROBOT = False
HAS_FEETECH = False

try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode, MotorCalibration
    HAS_LEROBOT = True
except ImportError:
    HAS_LEROBOT = False

# ── Calibration loading ──────────────────────────────────────────────────────
def load_calibration(robot_id="follower"):
    """Load calibration from lerobot cache."""
    cal_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower" / f"{robot_id}.json"
    if cal_path.exists():
        with open(cal_path) as f:
            return json.load(f)
    return None

# ── Trajectory maths ─────────────────────────────────────────────────────────
def _path_slopes(times, points):
    """Per-knot velocities for a shape-preserving (PCHIP) cubic through points.

    Zero at both ends (start and finish at rest). At interior knots a joint
    that keeps moving the same way keeps its speed instead of stopping, and
    a joint that reverses (or holds) gets zero speed, so the curve never
    overshoots a waypoint - important just above the table.
    """
    n, dims = len(points), len(points[0])
    slopes = [[0.0] * dims for _ in range(n)]
    for k in range(1, n - 1):
        h0, h1 = times[k] - times[k - 1], times[k + 1] - times[k]
        for j in range(dims):
            d0 = (points[k][j] - points[k - 1][j]) / h0
            d1 = (points[k + 1][j] - points[k][j]) / h1
            if d0 * d1 <= 0.0:
                continue
            w0, w1 = 2.0 * h1 + h0, h1 + 2.0 * h0
            slopes[k][j] = (w0 + w1) / (w0 / d0 + w1 / d1)
    return slopes


def _sample_path(traj, t):
    """Joint values of a queued trajectory at t seconds after its start."""
    times, points = traj["times"], traj["points"]
    if t <= 0.0:
        return list(points[0])
    if t >= times[-1]:
        return list(points[-1])
    k = max(i for i in range(len(times) - 1) if times[i] <= t)
    h = times[k + 1] - times[k]
    s = (t - times[k]) / h
    if len(points) == 2:
        # Single move: quintic, zero velocity and acceleration at both ends.
        e = s * s * s * (s * (s * 6.0 - 15.0) + 10.0)
        return [a + e * (b - a) for a, b in zip(points[0], points[1])]
    h00 = 2 * s ** 3 - 3 * s ** 2 + 1
    h10 = s ** 3 - 2 * s ** 2 + s
    h01 = -2 * s ** 3 + 3 * s ** 2
    h11 = s ** 3 - s ** 2
    m0, m1 = traj["slopes"][k], traj["slopes"][k + 1]
    return [h00 * a + h10 * h * va + h01 * b + h11 * h * vb
            for a, b, va, vb in zip(points[k], points[k + 1], m0, m1)]


# ── Hardware abstraction ─────────────────────────────────────────────────────
class HWArm:
    """
    Wraps real STS3215 servos.
    Motor IDs (factory default for SO-ARM101):
      1=shoulder_pan  2=shoulder_lift  3=elbow_flex
      4=wrist_flex    5=wrist_roll     6=gripper
    """
    MOTOR_IDS  = [1, 2, 3, 4, 5, 6]
    # Calibration offsets (radians) - adjust after physical calibration
    OFFSETS    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # STS3215: 4096 ticks = 360°, center = 2048
    TICKS_PER_RAD = 4096 / (2 * math.pi)

    def __init__(self, port, baud=1000000):
        self.port = port
        self.baud = baud
        self.bus  = None
        self.connected = False
        self._lock = threading.Lock()
        self._trajectory_lock = threading.Lock()
        self._trajectory = None
        self._trajectory_position = None
        self._trajectory_running = False
        # Trajectory duration and conservative fallback servo settings.
        self.goal_time_ms = 3500
        self.p_coefficient = SERVO_P_COEFFICIENT
        self.d_coefficient = SERVO_D_COEFFICIENT
        self.servo_acceleration = SERVO_ACCELERATION
        self.gripper_torque_limit = GRIPPER_TORQUE_LIMIT
        self.grip_detect = True
        self.grip_squeeze = GRIP_SQUEEZE_RAD
        self.grip_load_threshold = GRIP_LOAD_THRESHOLD
        self._grip = None
        self.grip_status = {"state": "idle"}
        self.move_speed = 45
        self.move_acceleration = 8

    def connect(self):
        if HAS_LEROBOT:
            self._connect_lerobot()
        elif HAS_FEETECH:
            self._connect_feetech()
        else:
            print("⚠  Neither lerobot nor feetech library found.")
            print("   Install with: pip install lerobot")
            print("   Running in MOCK mode — commands logged only.")
            self.connected = True   # mock

        if self.bus is not None:
            self._trajectory_position = self.read_joints()
            self._trajectory_running = True
            threading.Thread(target=self._trajectory_loop, daemon=True,
                             name="so101-smooth-motion").start()

    def _connect_lerobot(self):
        motors = {
            name: Motor(id=mid, model="sts3215", norm_mode=MotorNormMode.DEGREES)
            for name, mid in zip(JOINT_NAMES, self.MOTOR_IDS)
        }

        # Load calibration from file
        cal_data = load_calibration()
        if cal_data:
            calibration = {
                name: MotorCalibration(
                    id=cal_data[name]["id"],
                    drive_mode=cal_data[name]["drive_mode"],
                    homing_offset=cal_data[name]["homing_offset"],
                    range_min=cal_data[name]["range_min"],
                    range_max=cal_data[name]["range_max"]
                )
                for name in JOINT_NAMES
                if name in cal_data
            }
        else:
            # Fallback to defaults if calibration file not found
            calibration = {
                name: MotorCalibration(
                    id=mid,
                    drive_mode=0,
                    homing_offset=2048,
                    range_min=0,
                    range_max=4095
                )
                for name, mid in zip(JOINT_NAMES, self.MOTOR_IDS)
            }

        self.bus = FeetechMotorsBus(port=self.port, motors=motors, calibration=calibration)
        self.bus.connect()
        self.connected = True
        print(f"✓ Connected via lerobot to {self.port}")
        self._configure_servos()

    def _configure_servos(self):
        """Apply LeRobot's SO-101 servo tuning and plain position streaming.

        Written while torque stays on (the arm does not drop); the values take
        effect immediately and are re-applied on every connect.
        """
        settings = [
            ("P_Coefficient", self.p_coefficient),
            ("I_Coefficient", SERVO_I_COEFFICIENT),
            ("D_Coefficient", self.d_coefficient),
            ("Return_Delay_Time", 0),
            ("Acceleration", self.servo_acceleration),
            # No servo-side time/speed profile: the bridge streams the motion.
            ("Goal_Time", 0),
            ("Goal_Velocity", 0),
        ]
        failed = []
        with self._lock:
            for name in JOINT_NAMES:
                for register, value in settings:
                    # The gripper keeps its own gains: it grips by being driven
                    # past contact, so a lower P would weaken the grip.
                    if name == "gripper" and register.endswith("_Coefficient"):
                        continue
                    try:
                        self.bus.write(register, name, value, normalize=False)
                    except Exception as e:
                        failed.append(f"{name}.{register}: {e}")
        # Gripper current/torque protection. These live in the servo's EEPROM,
        # so (like LeRobot) write them with only the gripper's torque briefly
        # off; the arm joints keep holding.
        with self._lock:
            try:
                self.bus.disable_torque("gripper")
                for register, value in (("Max_Torque_Limit", self.gripper_torque_limit),
                                        ("Protection_Current", GRIPPER_PROTECTION_CURRENT),
                                        ("Overload_Torque", GRIPPER_OVERLOAD_TORQUE)):
                    self.bus.write(register, "gripper", value, normalize=False)
                self.bus.enable_torque("gripper")
                # Live (RAM) limit, so it applies without a power cycle.
                self.bus.write("Torque_Limit", "gripper", self.gripper_torque_limit,
                               normalize=False)
            except Exception as e:
                failed.append(f"gripper protection: {e}")
                try:
                    self.bus.enable_torque("gripper")
                except Exception:
                    pass
        if failed:
            print("  ⚠ Could not apply some servo settings:\n    " + "\n    ".join(failed[:6]))
        else:
            print(f"✓ Servo tuning: P={self.p_coefficient} I={SERVO_I_COEFFICIENT} "
                  f"D={self.d_coefficient} accel={self.servo_acceleration}, "
                  f"streaming at {TRAJECTORY_RATE_HZ:g} Hz; gripper torque limit "
                  f"{self.gripper_torque_limit / 10:g} %")

    def read_health(self) -> dict:
        """Load (% of max, signed), current (A), voltage (V), temperature (C)."""
        if not self.connected or self.bus is None or not HAS_LEROBOT:
            return {}
        readings = {}
        with self._lock:
            for register in ("Present_Load", "Present_Current", "Present_Voltage",
                             "Present_Temperature"):
                try:
                    readings[register] = self.bus.sync_read(register, normalize=False)
                except Exception as e:
                    readings[register] = {}
                    print(f"  Error reading {register}: {e}")
        health = {}
        for name in JOINT_NAMES:
            load = readings["Present_Load"].get(name)
            current = readings["Present_Current"].get(name)
            volts = readings["Present_Voltage"].get(name)
            temp = readings["Present_Temperature"].get(name)
            health[name] = {
                "load_pct": None if load is None else round(load / 10.0, 1),
                "current_a": None if current is None else round(current * 0.0065, 3),
                "voltage_v": None if volts is None else round(volts / 10.0, 1),
                "temperature_c": temp,
            }
        return health

    def _connect_feetech(self):
        self.bus = FeetechSMS_STS()
        r = self.bus.begin(self.baud, self.port)
        if r != 0:
            raise RuntimeError(f"Failed to open {self.port}")
        self.connected = True
        print(f"✓ Connected via feetech to {self.port}")

    def rad_to_ticks(self, rad, motor_id):
        return int(2048 + rad * self.TICKS_PER_RAD)

    def send_joints(self, joint_dict: dict):
        """Queue a smooth joint-space trajectory to the requested positions."""
        if not self.connected:
            return
        print(f"  HW cmd: { {k: round(v,3) for k,v in joint_dict.items()} } (time={self.goal_time_ms}ms)")
        if self.bus is None:
            return   # mock mode
        values = {name: float(value) for name, value in joint_dict.items() if name in JOINT_NAMES}
        if not values:
            return
        duration = max(0.25, min(10.0, self.goal_time_ms / 1000.0))
        self.send_path([values], [duration])

    def send_path(self, waypoints, durations):
        """Queue one continuous motion from the current pose through waypoints.

        Joints missing from a waypoint hold their previous value. Starts and
        ends at rest; passes through interior waypoints without stopping.
        """
        if not self.connected or self.bus is None or not waypoints:
            return
        names = sorted({n for w in waypoints for n in w if n in JOINT_NAMES},
                       key=JOINT_NAMES.index)
        if not names:
            return
        with self._trajectory_lock:
            if "gripper" in names:
                # An explicit gripper position (open, home) ends any grip.
                self._grip = None
                self.grip_status = {"state": "idle"}
            if self._trajectory_position is None:
                self._trajectory_position = self.read_joints()
            current = [float(self._trajectory_position.get(n, 0.0)) for n in names]
            points, times = [current], [0.0]
            for waypoint, duration in zip(waypoints, durations):
                previous = points[-1]
                points.append([float(waypoint.get(n, previous[k])) for k, n in enumerate(names)])
                times.append(times[-1] + max(0.05, min(10.0, float(duration))))
            self._trajectory = {
                "names": names,
                "times": times,
                "points": points,
                "slopes": _path_slopes(times, points),
                "start_time": time.monotonic(),
            }

    def _write_joints(self, joint_dict: dict, motion_time_ms: int):
        """Write one trajectory sample to the motor bus."""
        with self._lock:
            if HAS_LEROBOT:
                degrees_dict = {k: v * 180 / math.pi for k, v in joint_dict.items()}
                try:
                    # Position only: Goal_Time stays 0 (set at connect) so the
                    # servo follows the streamed trajectory directly.
                    self.bus.sync_write("Goal_Position", degrees_dict, normalize=True)
                except Exception as e:
                    print(f"  Error writing to motors: {e}")
            elif HAS_FEETECH:
                for i, name in enumerate(JOINT_NAMES):
                    if name in joint_dict:
                        ticks = self.rad_to_ticks(joint_dict[name], self.MOTOR_IDS[i])
                        self.bus.WritePosEx(self.MOTOR_IDS[i], ticks, self.move_speed, self.move_acceleration)

    def _trajectory_loop(self):
        """Stream the active trajectory as position goals at a fixed rate.

        Ticks are scheduled on absolute deadlines and samples are taken at the
        real elapsed time, so late ticks never distort the motion's shape.
        """
        interval_s = 1.0 / TRAJECTORY_RATE_HZ
        next_tick = time.monotonic()
        while self._trajectory_running:
            now = time.monotonic()
            with self._trajectory_lock:
                trajectory = self._trajectory
                if trajectory is not None:
                    elapsed = now - trajectory["start_time"]
                    values = _sample_path(trajectory, elapsed)
                    sample = dict(zip(trajectory["names"], values))
                    if elapsed >= trajectory["times"][-1]:
                        self._trajectory = None
                    if self._trajectory_position is None:
                        self._trajectory_position = {}
                    self._trajectory_position.update(sample)
                    self._write_joints(sample, 0)
                if self._grip is not None and self.grip_status.get("state") == "closing":
                    self._grip_tick(now)
            next_tick += interval_s
            remaining = next_tick - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()   # fell behind: do not burst

    def grip_close(self, target: float) -> None:
        """Close toward target until the jaws meet resistance, then hold
        a small squeeze past the contact point (see GRIP_* constants)."""
        if not self.connected or self.bus is None:
            return
        try:
            start, _ = self._read_gripper()
        except Exception:
            start = None
        with self._trajectory_lock:
            if start is None:
                start = float((self._trajectory_position or {}).get("gripper", target))
            if self._trajectory is not None and self._trajectory["names"] == ["gripper"]:
                self._trajectory = None   # replace a pending plain gripper move
            now = time.monotonic()
            self._grip = {"cmd": start, "target": float(target),
                          "dir": -1.0 if target < start else 1.0,
                          "last": now, "t0": now, "hits": 0}
            self.grip_status = {"state": "closing"}
        print(f"  Grip: closing from {start:.2f} toward {target:.2f} rad")

    def _read_gripper(self):
        """(position rad, signed load in 0.1 %) of the gripper servo."""
        with self._lock:
            degrees = self.bus.read("Present_Position", "gripper")
            load = self.bus.read("Present_Load", "gripper", normalize=False)
        return math.radians(float(degrees)), float(load)

    def _grip_tick(self, now: float) -> None:
        """One 50 Hz step of close-until-contact (trajectory lock held)."""
        g = self._grip
        dt, g["last"] = now - g["last"], now
        try:
            actual, load = self._read_gripper()
        except Exception as e:
            actual, load = None, 0.0
            print(f"  Grip read error: {e}")
        step = GRIP_SPEED_RAD_S * dt
        if g["dir"] < 0:
            g["cmd"] = max(g["target"], g["cmd"] - step)
        else:
            g["cmd"] = min(g["target"], g["cmd"] + step)
        lag = 0.0 if actual is None else (g["cmd"] - actual) * g["dir"]
        blocked = abs(load) >= self.grip_load_threshold or lag >= GRIP_LAG_THRESHOLD_RAD
        g["hits"] = g["hits"] + 1 if blocked else 0
        at_target = g["cmd"] == g["target"]
        near_closed = actual is not None and abs(actual - g["target"]) <= GRIP_EMPTY_MARGIN_RAD
        timed_out = now - g["t0"] > GRIP_TIMEOUT_S
        if g["hits"] >= GRIP_CONTACT_TICKS or (at_target and (near_closed or timed_out)):
            if actual is None or near_closed:
                state, hold = "empty", g["target"]
            else:
                state = "holding"
                hold = actual + g["dir"] * self.grip_squeeze
                hold = max(hold, g["target"]) if g["dir"] < 0 else min(hold, g["target"])
            g["cmd"] = hold
            self.grip_status = {
                "state": state,
                "contact_angle": None if actual is None else round(actual, 3),
                "hold_angle": round(hold, 3),
                "load_pct": round(load / 10.0, 1),
                "time_s": round(now - g["t0"], 2),
            }
            print(f"  Grip: {state} (contact {self.grip_status['contact_angle']} rad, "
                  f"load {self.grip_status['load_pct']} %, hold {hold:.2f} rad)")
        if self._trajectory_position is None:
            self._trajectory_position = {}
        self._trajectory_position["gripper"] = g["cmd"]
        self._write_joints({"gripper": g["cmd"]}, 0)

    def read_joints(self) -> dict:
        """Read current joint positions from real arm."""
        if not self.connected or self.bus is None:
            return {n: 0.0 for n in JOINT_NAMES}
        with self._lock:
            if HAS_LEROBOT:
                try:
                    pos = self.bus.sync_read("Present_Position")
                    # Convert from degrees to radians
                    return {name: pos[name] * math.pi / 180 for name in JOINT_NAMES if name in pos}
                except Exception as e:
                    print(f"  Error reading motors: {e}")
                    return {n: 0.0 for n in JOINT_NAMES}
            elif HAS_FEETECH:
                result = {}
                for i, name in enumerate(JOINT_NAMES):
                    pos, _ = self.bus.ReadPosEx(self.MOTOR_IDS[i])
                    result[name] = (pos - 2048) / self.TICKS_PER_RAD
                return result
        return {n: 0.0 for n in JOINT_NAMES}

    def set_gripper_torque(self, enabled: bool):
        """Enable or disable only the gripper motor torque for manual positioning."""
        if not self.connected or self.bus is None:
            return
        with self._lock:
            try:
                if HAS_LEROBOT and hasattr(self.bus, "enable_torque") and hasattr(self.bus, "disable_torque"):
                    if enabled:
                        self.bus.enable_torque("gripper")
                    else:
                        self.bus.disable_torque("gripper")
                elif HAS_FEETECH:
                    motor_id = self.MOTOR_IDS[JOINT_NAMES.index("gripper")]
                    self.bus.EnableTorque(motor_id, 1 if enabled else 0)
            except Exception as e:
                print(f"  Error setting gripper torque: {e}")

    def stop(self):
        """Cancel the active ramp and hold the current measured position."""
        if not self.connected:
            return
        current = self.read_joints()
        with self._trajectory_lock:
            self._trajectory = None
            if self._grip is not None and self.grip_status.get("state") == "holding":
                current["gripper"] = self._grip["cmd"]   # do not drop a held object
            elif self._grip is not None:
                self._grip = None
                self.grip_status = {"state": "idle"}
            if self._trajectory_position is None:
                self._trajectory_position = {}
            self._trajectory_position.update(current)
            if self.bus is not None:
                self._write_joints(current, 100)

    def disconnect(self):
        self._trajectory_running = False
        with self._trajectory_lock:
            self._trajectory = None
        if self.bus is not None:
            try:
                if HAS_LEROBOT: self.bus.disconnect()
            except: pass
        self.connected = False
        print("HW disconnected")

# ── HTTP Handler ─────────────────────────────────────────────────────────────
arm: HWArm = None

class HWHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def cors(self):
        self.send_header("Access-Control-Allow-Origin","*")
        self.send_header("Access-Control-Allow-Methods","GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers","Content-Type")

    def send_json(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self.send_header("Content-Length", str(len(body)))
        self.cors()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200); self.cors(); self.end_headers()

    def do_GET(self):
        if self.path == "/ping":
            self.send_json({"ok": True, "connected": arm.connected})
        elif self.path == "/joints":
            self.send_json({"joints": arm.read_joints()})
        elif self.path == "/health":
            self.send_json({"health": arm.read_health()})
        elif self.path == "/grip_status":
            self.send_json({"grip": arm.grip_status})
        elif self.path == "/motion_params":
            self.send_json({
                "time_ms": arm.goal_time_ms,
                "speed_legacy": arm.move_speed,
                "acceleration_legacy": arm.move_acceleration
            })
        elif self.path == "/home_pose":
            self.send_json({"pose": load_home_pose(), "file": str(HOME_POSE_FILE)})
        elif self.path == "/gripper_positions":
            self.send_json({"positions": load_gripper_positions(), "file": str(GRIPPER_POSITIONS_FILE)})
        else:
            self.send_json({"error":"not found"},404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length",0))
        body = json.loads(self.rfile.read(n)) if n else {}
        if self.path == "/move":
            joints = body.get("joints", {})
            arm.send_joints(joints)
            self.send_json({"ok":True})
        elif self.path == "/move_path":
            waypoints = body.get("waypoints", [])
            durations = body.get("durations", [])
            if (not isinstance(waypoints, list) or not waypoints
                    or len(durations) != len(waypoints)):
                self.send_json({"ok": False, "error": "need waypoints and one duration each"}, 400)
                return
            arm.send_path(waypoints, durations)
            self.send_json({"ok": True, "points": len(waypoints)})
        elif self.path == "/gripper":
            val = body.get("value", 0)
            target = gripper_value_to_angle(val)
            # Closing commands stop on contact unless plain position is asked for.
            if arm.grip_detect and float(val) >= 0.5 and body.get("mode") != "position":
                arm.grip_close(target)
            else:
                arm.send_joints({"gripper": target})
            self.send_json({"ok":True})
        elif self.path == "/home":
            pose = body.get("pose") if isinstance(body.get("pose"), dict) else load_home_pose()
            arm.send_joints(_validated_home_pose(pose))
            self.send_json({"ok":True})
        elif self.path == "/stop":
            arm.stop()
            self.send_json({"ok": True})
        elif self.path == "/gripper_torque":
            enabled = bool(body.get("enabled", True))
            arm.set_gripper_torque(enabled)
            self.send_json({"ok": True, "enabled": enabled})
        elif self.path == "/home_pose":
            try:
                saved = save_home_pose(body.get("pose", {}))
            except (TypeError, ValueError, OSError) as e:
                self.send_json({"ok": False, "error": str(e)}, 400)
                return
            self.send_json({"ok": True, "pose": saved, "file": str(HOME_POSE_FILE)})
        elif self.path == "/gripper_positions":
            try:
                name = str(body["name"]).strip().lower()
                if name not in ("open", "close"):
                    raise ValueError("Gripper position name must be open or close")
                if "value" in body:
                    value = float(body["value"])
                else:
                    value = arm.read_joints()["gripper"]
                positions = load_gripper_positions()
                positions[name] = value
                saved = save_gripper_positions(positions)
            except (KeyError, TypeError, ValueError, OSError) as e:
                self.send_json({"ok": False, "error": str(e)}, 400)
                return
            self.send_json({"ok": True, "positions": saved, "file": str(GRIPPER_POSITIONS_FILE)})
        elif self.path == "/set_motion_time":
            # Set motion time in milliseconds (higher = slower, smoother)
            time_ms = body.get("time_ms", 3500)
            arm.goal_time_ms = max(0, min(10000, time_ms))  # Cap at 10 seconds
            self.send_json({"ok": True, "time_ms": arm.goal_time_ms})
        elif self.path == "/set_speed":
            # Legacy endpoint - converts speed to time_ms
            # Speed 0-1000 maps to time_ms 3000-100
            speed = body.get("speed", 100)
            speed = max(1, min(1000, speed))
            arm.goal_time_ms = int(3000 - (speed - 1) * 2900 / 999)
            self.send_json({"ok": True, "speed": speed, "time_ms": arm.goal_time_ms})
        elif self.path == "/set_acceleration":
            # Legacy endpoint - for Feetech fallback only
            accel = body.get("acceleration", 30)
            arm.move_acceleration = max(0, min(254, accel))
            self.send_json({"ok": True, "acceleration": arm.move_acceleration})
        else:
            self.send_json({"error":"not found"},404)

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    global arm
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",  default="/dev/ttyUSB0")
    ap.add_argument("--baud",  type=int, default=1000000)
    ap.add_argument("--hwport",type=int, default=PORT)
    ap.add_argument("--p-gain", type=int, default=SERVO_P_COEFFICIENT,
                    help="servo position P gain (LeRobot SO-101: 16; lower = softer, less vibration)")
    ap.add_argument("--d-gain", type=int, default=SERVO_D_COEFFICIENT,
                    help="servo position D gain (LeRobot SO-101: 32; damps oscillation)")
    ap.add_argument("--servo-accel", type=int, default=SERVO_ACCELERATION,
                    help="servo acceleration limit 1..254 (lower smooths small steps but lags)")
    ap.add_argument("--gripper-torque-limit", type=int, default=GRIPPER_TORQUE_LIMIT,
                    help="gripper torque limit in 0.1 %% units (LeRobot SO-101: 500 = 50 %%); "
                         "raise if objects slip, lower if the arm jitters while holding")
    ap.add_argument("--no-grip-detect", action="store_true",
                    help="close the gripper to its set position instead of stopping on contact")
    ap.add_argument("--grip-squeeze-deg", type=float, default=math.degrees(GRIP_SQUEEZE_RAD),
                    help="how far past the contact point to hold (default 5; more = firmer)")
    ap.add_argument("--grip-load-threshold", type=float, default=GRIP_LOAD_THRESHOLD / 10.0,
                    help="gripper load %% that counts as contact (default 30)")
    ap.add_argument("--log-health", type=float, default=0.0, metavar="SECONDS",
                    help="print every servo's load/current/voltage this often (0 = off)")
    args = ap.parse_args()

    arm = HWArm(args.port, args.baud)
    arm.p_coefficient = max(1, min(254, args.p_gain))
    arm.d_coefficient = max(0, min(254, args.d_gain))
    arm.servo_acceleration = max(1, min(254, args.servo_accel))
    arm.gripper_torque_limit = max(50, min(1000, args.gripper_torque_limit))
    arm.grip_detect = not args.no_grip_detect
    arm.grip_squeeze = math.radians(max(0.0, min(30.0, args.grip_squeeze_deg)))
    arm.grip_load_threshold = max(5.0, min(100.0, args.grip_load_threshold)) * 10.0
    arm.connect()
    if args.log_health > 0:
        def log_health():
            while arm.connected:
                health = arm.read_health()
                if health:
                    print("  health: " + "  ".join(
                        f"{n.split('_')[0][:5]} {h['load_pct']}% {h['current_a']}A"
                        for n, h in health.items()) +
                        f"  | {min(h['voltage_v'] or 99 for h in health.values())}V min")
                time.sleep(args.log_health)
        threading.Thread(target=log_health, daemon=True).start()

    srv = HTTPServer(("127.0.0.1", args.hwport), HWHandler)
    print(f"✓ HW Bridge running on http://127.0.0.1:{args.hwport}")
    print(f"  Serial port: {args.port}  Baud: {args.baud}")
    print(f"  Connected: {arm.connected}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        arm.disconnect()

if __name__ == "__main__":
    main()
