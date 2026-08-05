"""
Item Segregator - Real-Time Approach
=====================================
Uses computer vision to detect item color (Red, Blue, White) in a fixed
pickup zone, then commands the SO101 arm to predefined joint positions to
pick the item and place it in the correct bin.

NO pre-recorded datasets are needed.  Instead, the robot moves through a
sequence of waypoints (predefined joint angles) for:
  1. Home position
  2. Pickup hover -> Pickup grasp -> Lift
  3. Move to the target bin -> Release -> Retreat
  4. Return to Home

Color Mapping:
  - Red   -> Bin 1
  - Blue  -> Bin 2
  - White -> Discard

Hardware: SO101 Follower arm via LeRobot

IMPORTANT: You MUST calibrate the joint angles for your physical setup.
           See the WAYPOINTS section below and update the values to match
           your arm's reach and bin placement.
"""

import time
import cv2
import numpy as np
from pathlib import Path

from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.utils.robot_utils import busy_wait


# =====================================================================
# WAYPOINTS  –  EDIT THESE TO MATCH YOUR PHYSICAL SETUP
# =====================================================================
# Each waypoint is a dict mapping joint names to target angles (degrees).
# The SO101 follower typically has these joints:
#   "shoulder_pan", "shoulder_lift", "elbow_flex",
#   "wrist_flex", "wrist_roll", "gripper"
#
# Adjust the values below after manually posing the arm to each position
# and reading the joint angles.
# =====================================================================

# Gripper open/close values (adjust for your gripper)
GRIPPER_OPEN = 50.0
GRIPPER_CLOSED = -30.0

# Home / neutral position
HOME = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_OPEN,
}

# Hover above pickup zone (gripper open)
PICKUP_HOVER = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -40.0,
    "elbow_flex": -30.0,
    "wrist_flex": -20.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_OPEN,
}

# Grasp position (lower into pickup zone, gripper open -> then close)
PICKUP_GRASP = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -50.0,
    "elbow_flex": -40.0,
    "wrist_flex": -25.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_OPEN,
}

# Lift after grasping
PICKUP_LIFT = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -30.0,
    "elbow_flex": -20.0,
    "wrist_flex": -15.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_CLOSED,
}

# Bin 1 (Red) – hover above bin then release
BIN1_HOVER = {
    "shoulder_pan": -60.0,
    "shoulder_lift": -30.0,
    "elbow_flex": -20.0,
    "wrist_flex": -10.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_CLOSED,
}

BIN1_RELEASE = {
    "shoulder_pan": -60.0,
    "shoulder_lift": -40.0,
    "elbow_flex": -30.0,
    "wrist_flex": -15.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_OPEN,
}

# Bin 2 (Blue) – hover above bin then release
BIN2_HOVER = {
    "shoulder_pan": 60.0,
    "shoulder_lift": -30.0,
    "elbow_flex": -20.0,
    "wrist_flex": -10.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_CLOSED,
}

BIN2_RELEASE = {
    "shoulder_pan": 60.0,
    "shoulder_lift": -40.0,
    "elbow_flex": -30.0,
    "wrist_flex": -15.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_OPEN,
}

# Discard (White) – e.g. push/drop off the side
DISCARD_HOVER = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -20.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_CLOSED,
}

DISCARD_RELEASE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -20.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": GRIPPER_OPEN,
}

# Motion sequences per destination
PICK_AND_PLACE_SEQUENCES = {
    "red": [
        ("Move to pickup hover", PICKUP_HOVER, 1.0),
        ("Lower to grasp", PICKUP_GRASP, 0.8),
        ("Close gripper", {**PICKUP_GRASP, "gripper": GRIPPER_CLOSED}, 0.5),
        ("Lift item", PICKUP_LIFT, 0.8),
        ("Move to Bin 1", BIN1_HOVER, 1.2),
        ("Lower into Bin 1", BIN1_RELEASE, 0.6),
        ("Open gripper (release)", {**BIN1_RELEASE, "gripper": GRIPPER_OPEN}, 0.5),
        ("Retreat from Bin 1", BIN1_HOVER, 0.6),
        ("Return home", HOME, 1.0),
    ],
    "blue": [
        ("Move to pickup hover", PICKUP_HOVER, 1.0),
        ("Lower to grasp", PICKUP_GRASP, 0.8),
        ("Close gripper", {**PICKUP_GRASP, "gripper": GRIPPER_CLOSED}, 0.5),
        ("Lift item", PICKUP_LIFT, 0.8),
        ("Move to Bin 2", BIN2_HOVER, 1.2),
        ("Lower into Bin 2", BIN2_RELEASE, 0.6),
        ("Open gripper (release)", {**BIN2_RELEASE, "gripper": GRIPPER_OPEN}, 0.5),
        ("Retreat from Bin 2", BIN2_HOVER, 0.6),
        ("Return home", HOME, 1.0),
    ],
    "white": [
        ("Move to pickup hover", PICKUP_HOVER, 1.0),
        ("Lower to grasp", PICKUP_GRASP, 0.8),
        ("Close gripper", {**PICKUP_GRASP, "gripper": GRIPPER_CLOSED}, 0.5),
        ("Lift item", PICKUP_LIFT, 0.8),
        ("Move to discard", DISCARD_HOVER, 1.2),
        ("Release (discard)", DISCARD_RELEASE, 0.6),
        ("Open gripper", {**DISCARD_RELEASE, "gripper": GRIPPER_OPEN}, 0.5),
        ("Retreat", DISCARD_HOVER, 0.6),
        ("Return home", HOME, 1.0),
    ],
}


# =====================================================================
# Color Detection Vision System  (same as replay version)
# =====================================================================

class ItemColorDetector:
    """Computer vision system for detecting item color in a fixed pickup zone."""

    COLOR_RANGES = {
        "red": {
            "lower1": np.array([0, 100, 80]),
            "upper1": np.array([10, 255, 255]),
            "lower2": np.array([160, 100, 80]),
            "upper2": np.array([180, 255, 255]),
        },
        "blue": {
            "lower": np.array([100, 100, 80]),
            "upper": np.array([130, 255, 255]),
        },
        "white": {
            "lower": np.array([0, 0, 180]),
            "upper": np.array([180, 50, 255]),
        },
    }

    MIN_CONTOUR_AREA = 500

    def __init__(self, camera_id: int = 0):
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise ValueError(f"Could not open camera {camera_id}")
        self.roi_corners = None
        self.roi_size = 300

    def calibrate_pickup_zone(self) -> bool:
        print("\n" + "=" * 60)
        print("PICKUP ZONE CALIBRATION")
        print("=" * 60)
        print("Click the 4 corners of the pickup zone in order:")
        print("  1. Top-left   2. Top-right")
        print("  3. Bottom-right   4. Bottom-left")
        print("Keys: 'r' = reset | 'c' = confirm | 'q' = quit")
        print("=" * 60 + "\n")

        corners: list = []

        def _mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 4:
                corners.append([x, y])
                print(f"  Corner {len(corners)}/4 at ({x}, {y})")

        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", _mouse_cb)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            disp = frame.copy()
            for i, c in enumerate(corners):
                cv2.circle(disp, tuple(c), 5, (0, 255, 0), -1)
                cv2.putText(disp, str(i + 1), (c[0] + 10, c[1] + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if len(corners) > 1:
                for i in range(len(corners)):
                    cv2.line(disp, tuple(corners[i]),
                             tuple(corners[(i + 1) % len(corners)]),
                             (0, 255, 0), 2)
            cv2.imshow("Calibration", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                corners.clear()
                print("  Reset corners")
            elif key == ord("c") and len(corners) == 4:
                self.roi_corners = np.array(corners, dtype=np.float32)
                print("Calibration complete!")
                break
            elif key == ord("q"):
                cv2.destroyAllWindows()
                return False

        cv2.destroyAllWindows()
        return True

    def _warp_roi(self, frame):
        if self.roi_corners is None:
            return None
        dst = np.array([[0, 0], [self.roi_size, 0],
                        [self.roi_size, self.roi_size],
                        [0, self.roi_size]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(self.roi_corners, dst)
        return cv2.warpPerspective(frame, M, (self.roi_size, self.roi_size))

    def _get_color_mask(self, hsv, color_name: str):
        cfg = self.COLOR_RANGES[color_name]
        if color_name == "red":
            return (cv2.inRange(hsv, cfg["lower1"], cfg["upper1"])
                    | cv2.inRange(hsv, cfg["lower2"], cfg["upper2"]))
        return cv2.inRange(hsv, cfg["lower"], cfg["upper"])

    def detect_color(self) -> str | None:
        ret, frame = self.cap.read()
        if not ret:
            return None
        warped = self._warp_roi(frame)
        if warped is None:
            return None

        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        best_color, best_area = None, 0
        for name in self.COLOR_RANGES:
            mask = self._get_color_mask(hsv, name)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            area = sum(cv2.contourArea(c) for c in contours
                       if cv2.contourArea(c) > self.MIN_CONTOUR_AREA)
            if area > best_area:
                best_area = area
                best_color = name

        return best_color if best_area >= self.MIN_CONTOUR_AREA else None

    def detect_color_stable(self, required_frames: int = 10,
                            timeout: float = 10.0) -> str | None:
        start = time.time()
        last_color, streak = None, 0

        while (time.time() - start) < timeout:
            color = self.detect_color()
            if color is not None and color == last_color:
                streak += 1
                if streak >= required_frames:
                    return color
            else:
                last_color = color
                streak = 1 if color else 0

            ret, frame = self.cap.read()
            if ret:
                disp = frame.copy()
                if self.roi_corners is not None:
                    cv2.polylines(disp, [self.roi_corners.astype(np.int32)],
                                  True, (0, 255, 0), 2)
                lbl = f"Detecting: {last_color or '---'}  ({streak}/{required_frames})"
                cv2.putText(disp, lbl, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("Pickup Zone", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        return None

    def show_live(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        disp = frame.copy()
        if self.roi_corners is not None:
            cv2.polylines(disp, [self.roi_corners.astype(np.int32)],
                          True, (0, 255, 0), 2)
        cv2.imshow("Pickup Zone", disp)
        cv2.waitKey(1)

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


# =====================================================================
# Real-Time Robot Control
# =====================================================================

class RobotController:
    """Send joint-position commands to the SO101 arm in real time."""

    def __init__(self, port: str = "/dev/ttyACM1", fps: int = 30,
                 interpolation_steps: int = 30):
        """
        Args:
            port: Serial port for SO101.
            fps: Control loop frequency (Hz).
            interpolation_steps: Number of intermediate steps for smooth
                                 motion between waypoints.
        """
        self.port = port
        self.fps = fps
        self.interpolation_steps = interpolation_steps
        self.robot = None
        self.joint_names = list(HOME.keys())

    def connect(self):
        config = SO101FollowerConfig(port=self.port)
        self.robot = SO101Follower(config)
        self.robot.connect()
        print("  Robot connected")

    def disconnect(self):
        if self.robot is not None:
            try:
                self.robot.disconnect()
                print("  Robot disconnected")
            except Exception:
                pass
            self.robot = None

    def move_to(self, target: dict, duration: float = 1.0):
        """
        Smoothly interpolate from current position to *target* over
        *duration* seconds.

        If current position is unknown (first call), snaps directly.
        """
        if self.robot is None:
            raise RuntimeError("Robot not connected")

        steps = max(1, int(duration * self.fps))

        # Read current position (fallback to target if unavailable)
        try:
            current_obs = self.robot.get_observation()
            current = {
                name: float(current_obs["observation.state"][i])
                for i, name in enumerate(self.joint_names)
            }
        except Exception:
            current = dict(target)  # snap

        for step in range(1, steps + 1):
            t0 = time.perf_counter()
            alpha = step / steps
            interpolated = {
                name: current[name] + alpha * (target[name] - current[name])
                for name in self.joint_names
            }
            self.robot.send_action(interpolated)
            busy_wait(1.0 / self.fps - (time.perf_counter() - t0))

    def execute_sequence(self, sequence: list[tuple[str, dict, float]]):
        """
        Execute a list of (label, target_joints, duration) waypoints.
        """
        for label, target, duration in sequence:
            print(f"    -> {label}")
            self.move_to(target, duration)


# =====================================================================
# Main Segregator (Real-Time Approach)
# =====================================================================

COLOR_LABEL = {
    "red":   "Bin 1 (Red)",
    "blue":  "Bin 2 (Blue)",
    "white": "Discard (White)",
}


class ItemSegregatorRealtime:
    """
    Detects item color via vision, then drives the arm through predefined
    waypoints in real time to pick the item and place it in the correct bin.
    """

    def __init__(self, port: str = "/dev/ttyACM1", camera_id: int = 0,
                 fps: int = 30, interpolation_steps: int = 30):
        self.port = port
        self.controller = RobotController(port, fps, interpolation_steps)
        self.detector = ItemColorDetector(camera_id)
        self.history: list[dict] = []

    def calibrate(self) -> bool:
        return self.detector.calibrate_pickup_zone()

    def segregate_one(self) -> bool:
        print("\nPlace an item in the pickup zone and press ENTER to detect...")
        input()

        print("Detecting item color...")
        color = self.detector.detect_color_stable(required_frames=10,
                                                   timeout=15)
        if color is None:
            print("No item detected. Ensure the item is in the pickup zone.")
            return False

        label = COLOR_LABEL[color]
        sequence = PICK_AND_PLACE_SEQUENCES[color]

        print(f"\n  Detected color : {color.upper()}")
        print(f"  Destination    : {label}")
        print("  Executing motion sequence...\n")

        try:
            self.controller.execute_sequence(sequence)
            success = True
            print(f"\n  Item ({color}) sent to {label} successfully!")
        except Exception as e:
            success = False
            print(f"\n  Motion failed: {e}")
            import traceback
            traceback.print_exc()

        self.history.append({
            "color": color,
            "destination": label,
            "success": success,
        })
        return True

    def run(self):
        print("=" * 60)
        print("  ITEM SEGREGATOR  -  Real-Time Approach")
        print("=" * 60)
        print(f"  Red   -> Bin 1")
        print(f"  Blue  -> Bin 2")
        print(f"  White -> Discard")
        print("=" * 60)

        if not self.calibrate():
            print("Calibration failed. Exiting.")
            return

        # Connect robot
        try:
            self.controller.connect()
        except Exception as e:
            print(f"Could not connect to robot: {e}")
            self.detector.release()
            return

        # Go to home position
        print("\nMoving to home position...")
        try:
            self.controller.move_to(HOME, duration=2.0)
        except Exception as e:
            print(f"Failed to reach home: {e}")

        try:
            while True:
                self.detector.show_live()
                self.segregate_one()

                cont = input("\nProcess another item? (yes/no): ").strip().lower()
                if cont not in ("yes", "y"):
                    break
        finally:
            # Always disconnect cleanly
            print("\nMoving to home before shutdown...")
            try:
                self.controller.move_to(HOME, duration=1.5)
            except Exception:
                pass
            self.controller.disconnect()
            self._print_summary()
            self.detector.release()

    def _print_summary(self):
        print("\n" + "=" * 60)
        print("  SESSION SUMMARY")
        print("=" * 60)
        for i, entry in enumerate(self.history, 1):
            status = "OK" if entry["success"] else "FAIL"
            print(f"  {i}. {entry['color'].upper():6s} -> "
                  f"{entry['destination']}  [{status}]")
        print("=" * 60 + "\n")


# =====================================================================
# Waypoint Calibration Utility
# =====================================================================

def calibrate_waypoints(port: str = "/dev/ttyACM1"):
    """
    Interactive helper to record joint positions for waypoint calibration.

    Procedure:
      1. Connect the robot.
      2. Manually move the arm to each desired position.
      3. Press ENTER to record current joint angles.
      4. Copy the printed dict into the WAYPOINTS section above.
    """
    config = SO101FollowerConfig(port=port)
    robot = SO101Follower(config)
    robot.connect()
    print("Robot connected for waypoint calibration.")
    print("Manually move the arm, then press ENTER to record.\n")

    joint_names = list(HOME.keys())

    try:
        while True:
            label = input("Waypoint label (or 'q' to quit): ").strip()
            if label.lower() == "q":
                break

            input("  Position the arm, then press ENTER to record...")
            obs = robot.get_observation()
            waypoint = {
                name: round(float(obs["observation.state"][i]), 2)
                for i, name in enumerate(joint_names)
            }

            print(f"\n  {label} = {{")
            for k, v in waypoint.items():
                print(f'      "{k}": {v},')
            print("  }\n")
    finally:
        robot.disconnect()
        print("Robot disconnected.")


# =====================================================================
# Entry Point
# =====================================================================

def main():
    print("=" * 60)
    print("  Item Segregator  -  Real-Time Approach")
    print("=" * 60)

    print("\nOptions:")
    print("  1. Run item segregator")
    print("  2. Calibrate waypoints (record joint positions)")
    choice = input("Select (1 or 2, default: 1): ").strip()

    port = input(
        "Enter robot serial port (default: /dev/ttyACM1): "
    ).strip() or "/dev/ttyACM1"

    if choice == "2":
        calibrate_waypoints(port)
        return

    camera_input = input("Enter camera ID (default: 0): ").strip()
    camera_id = 0
    if camera_input:
        try:
            camera_id = int(camera_input)
        except ValueError:
            print("Invalid camera ID, using default (0)")

    fps_input = input("Control loop FPS (default: 30): ").strip()
    fps = 30
    if fps_input:
        try:
            fps = int(fps_input)
        except ValueError:
            print("Invalid FPS, using default (30)")

    try:
        seg = ItemSegregatorRealtime(port, camera_id, fps)
        seg.run()
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
