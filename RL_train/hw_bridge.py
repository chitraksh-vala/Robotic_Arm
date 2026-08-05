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
  POST /gripper            { value: 0..1 }
  POST /home
  POST /set_motion_time    { time_ms: 0..10000 }  - Time to reach position (higher = slower, default: 1500ms)
  POST /set_speed          { speed: 0..1000 }     - Legacy, converts to time_ms
  POST /set_acceleration   { acceleration: 0..254 } - For Feetech fallback only
  GET  /joints             → current joint positions
  GET  /motion_params      → current motion settings
  GET  /ping               → connection status

Motion Control for STS3215:
  The servos use Goal_Time (milliseconds) to control motion speed.
  Higher values = slower, smoother motion
  Default: 1500ms (1.5 seconds to reach target)
  Range: 0-10000ms
"""
import argparse, json, math, time, threading, sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

JOINT_NAMES = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
PORT = 7843

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
        # Motion smoothing parameters for STS3215 servos
        self.goal_time_ms = 1500    # Time in milliseconds to reach target (higher = slower, smoother)
        self.move_speed = 100       # For Feetech fallback (range: 0-1000)
        self.move_acceleration = 30 # For Feetech fallback (range: 0-254)

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
        """Send joint angle dict {name: rad} to real arm with smooth motion."""
        if not self.connected:
            return
        print(f"  HW cmd: { {k: round(v,3) for k,v in joint_dict.items()} } (time={self.goal_time_ms}ms)")
        if self.bus is None:
            return   # mock mode
        with self._lock:
            if HAS_LEROBOT:
                # Convert radians to degrees for the motor bus
                degrees_dict = {k: v * 180 / math.pi for k, v in joint_dict.items()}
                try:
                    # For STS3215, use Goal_Time to control motion speed/smoothness
                    # Goal_Time is in milliseconds - higher values = slower, smoother motion
                    time_dict = {k: self.goal_time_ms for k in degrees_dict.keys()}
                    self.bus.sync_write("Goal_Time", time_dict)
                    # Now send the position
                    self.bus.sync_write("Goal_Position", degrees_dict, normalize=True)
                except Exception as e:
                    print(f"  Error writing to motors: {e}")
            elif HAS_FEETECH:
                for i, name in enumerate(JOINT_NAMES):
                    if name in joint_dict:
                        ticks = self.rad_to_ticks(joint_dict[name], self.MOTOR_IDS[i])
                        # Use configurable speed and acceleration instead of hardcoded values
                        self.bus.WritePosEx(self.MOTOR_IDS[i], ticks, self.move_speed, self.move_acceleration)

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

    def disconnect(self):
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
        elif self.path == "/motion_params":
            self.send_json({
                "time_ms": arm.goal_time_ms,
                "speed_legacy": arm.move_speed,
                "acceleration_legacy": arm.move_acceleration
            })
        else:
            self.send_json({"error":"not found"},404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length",0))
        body = json.loads(self.rfile.read(n)) if n else {}
        if self.path == "/move":
            joints = body.get("joints", {})
            arm.send_joints(joints)
            self.send_json({"ok":True})
        elif self.path == "/gripper":
            val = body.get("value", 0)
            arm.send_joints({"gripper": val * 1.7453})
            self.send_json({"ok":True})
        elif self.path == "/home":
            arm.send_joints({n:0.0 for n in JOINT_NAMES})
            self.send_json({"ok":True})
        elif self.path == "/set_motion_time":
            # Set motion time in milliseconds (higher = slower, smoother)
            time_ms = body.get("time_ms", 1500)
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
    args = ap.parse_args()

    arm = HWArm(args.port, args.baud)
    arm.connect()

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
