"""
Item Segregator - Replay Approach
=================================
Uses computer vision to detect item color (Red, Blue, White) in a fixed
pickup zone, then replays a pre-recorded LeRobot episode to move the item
to the correct bin.

Color Mapping:
  - Red   -> Bin 1
  - Blue  -> Bin 2
  - White -> Discard

Dataset Folder Structure Expected:
  <base_path>/
    record_bin1/       # Episodes for picking and placing into Bin 1 (Red)
    record_bin2/       # Episodes for picking and placing into Bin 2 (Blue)
    record_discard/    # Episodes for picking and discarding (White)

Each folder should contain LeRobot dataset episodes (episode indices 0, 1, 2, ...).
The system cycles through episode indices for variety.

Hardware: SO101 Follower arm via LeRobot
"""

import time
import cv2
import numpy as np
from collections import deque
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.utils.robot_utils import busy_wait


# ---------------------------------------------------------------------------
# Color Detection Vision System
# ---------------------------------------------------------------------------

class ItemColorDetector:
    """Computer vision system for detecting item color in a fixed pickup zone."""

    # HSV ranges for each target color
    # (These may need tuning for your lighting conditions)
    COLOR_RANGES = {
        "red": {
            # Red wraps around the hue wheel, so two ranges are needed
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

    # Minimum contour area (in pixels) to count as a valid detection
    MIN_CONTOUR_AREA = 500

    def __init__(self, camera_id: int = 0):
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise ValueError(f"Could not open camera {camera_id}")

        # Region-of-interest corners (set during calibration)
        self.roi_corners = None
        self.roi_size = 300  # warped ROI square size

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def calibrate_pickup_zone(self) -> bool:
        """
        Let the user click 4 corners of the pickup zone.
        Order: top-left, top-right, bottom-right, bottom-left.
        Press 'c' to confirm, 'r' to reset, 'q' to quit.
        """
        print("\n" + "=" * 60)
        print("PICKUP ZONE CALIBRATION")
        print("=" * 60)
        print("Click the 4 corners of the pickup zone in order:")
        print("  1. Top-left")
        print("  2. Top-right")
        print("  3. Bottom-right")
        print("  4. Bottom-left")
        print("Keys: 'r' = reset | 'c' = confirm | 'q' = quit")
        print("=" * 60 + "\n")

        corners: list = []

        def _mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 4:
                corners.append([x, y])
                print(f"  Corner {len(corners)}/4 selected at ({x}, {y})")

        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", _mouse_cb)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            display = frame.copy()
            for i, c in enumerate(corners):
                cv2.circle(display, tuple(c), 5, (0, 255, 0), -1)
                cv2.putText(display, str(i + 1), (c[0] + 10, c[1] + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if len(corners) > 1:
                for i in range(len(corners)):
                    cv2.line(display, tuple(corners[i]),
                             tuple(corners[(i + 1) % len(corners)]),
                             (0, 255, 0), 2)

            cv2.imshow("Calibration", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("r"):
                corners.clear()
                print("  Reset corners")
            elif key == ord("c") and len(corners) == 4:
                self.roi_corners = np.array(corners, dtype=np.float32)
                print("Calibration complete!")
                break
            elif key == ord("q"):
                print("Calibration cancelled")
                cv2.destroyAllWindows()
                return False

        cv2.destroyAllWindows()
        return True

    # ------------------------------------------------------------------
    # Image helpers
    # ------------------------------------------------------------------

    def _warp_roi(self, frame):
        """Perspective-transform the pickup zone to a square image."""
        if self.roi_corners is None:
            return None
        dst = np.array([
            [0, 0],
            [self.roi_size, 0],
            [self.roi_size, self.roi_size],
            [0, self.roi_size],
        ], dtype=np.float32)
        M = cv2.getPerspectiveTransform(self.roi_corners, dst)
        return cv2.warpPerspective(frame, M, (self.roi_size, self.roi_size))

    def _get_color_mask(self, hsv, color_name: str):
        """Return a binary mask for the given color name."""
        cfg = self.COLOR_RANGES[color_name]
        if color_name == "red":
            m1 = cv2.inRange(hsv, cfg["lower1"], cfg["upper1"])
            m2 = cv2.inRange(hsv, cfg["lower2"], cfg["upper2"])
            return m1 | m2
        return cv2.inRange(hsv, cfg["lower"], cfg["upper"])

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def detect_color(self) -> str | None:
        """
        Capture a frame, warp the pickup zone, and determine the dominant
        item color.

        Returns:
            "red", "blue", "white", or None if nothing significant detected.
        """
        ret, frame = self.cap.read()
        if not ret:
            return None

        warped = self._warp_roi(frame)
        if warped is None:
            return None

        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        # Apply a slight blur to reduce noise
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        best_color = None
        best_area = 0

        for color_name in self.COLOR_RANGES:
            mask = self._get_color_mask(hsv, color_name)

            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            area = sum(cv2.contourArea(c) for c in contours
                       if cv2.contourArea(c) > self.MIN_CONTOUR_AREA)

            if area > best_area:
                best_area = area
                best_color = color_name

        if best_area < self.MIN_CONTOUR_AREA:
            return None

        return best_color

    def detect_color_stable(self, required_frames: int = 10,
                            timeout: float = 10.0) -> str | None:
        """
        Wait until the same color is detected for *required_frames*
        consecutive reads, or until *timeout* seconds elapse.
        """
        start = time.time()
        last_color = None
        streak = 0

        while (time.time() - start) < timeout:
            color = self.detect_color()

            if color is not None and color == last_color:
                streak += 1
                if streak >= required_frames:
                    return color
            else:
                last_color = color
                streak = 1 if color is not None else 0

            # Show live feed
            ret, frame = self.cap.read()
            if ret:
                disp = frame.copy()
                if self.roi_corners is not None:
                    cv2.polylines(disp,
                                  [self.roi_corners.astype(np.int32)],
                                  True, (0, 255, 0), 2)
                label = f"Detecting: {last_color or '---'}  ({streak}/{required_frames})"
                cv2.putText(disp, label, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("Pickup Zone", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        return None

    def show_live(self):
        """Show the camera feed with ROI overlay (non-blocking single frame)."""
        ret, frame = self.cap.read()
        if not ret:
            return
        disp = frame.copy()
        if self.roi_corners is not None:
            cv2.polylines(disp,
                          [self.roi_corners.astype(np.int32)],
                          True, (0, 255, 0), 2)
        cv2.imshow("Pickup Zone", disp)
        cv2.waitKey(1)

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Robot Episode Replay
# ---------------------------------------------------------------------------

def replay_episode(base_path: str, folder_name: str, episode_idx: int,
                   port: str = "/dev/ttyACM1") -> bool:
    """
    Replay a single episode from a LeRobot dataset folder.

    Args:
        base_path:    Root directory containing dataset folders.
        folder_name:  Sub-folder name (e.g. "record_bin1").
        episode_idx:  Episode index inside the dataset.
        port:         Serial port for the SO101 follower.

    Returns:
        True on success, False on failure.
    """
    dataset_path = Path(base_path) / folder_name

    if not dataset_path.exists():
        print(f"Dataset path does not exist: {dataset_path}")
        return False

    print(f"\n  Loading dataset from: {dataset_path}")
    print(f"  Episode index: {episode_idx}")

    robot = None
    try:
        robot_config = SO101FollowerConfig(port=port)
        robot = SO101Follower(robot_config)
        robot.connect()
        print("  Robot connected")

        dataset = LeRobotDataset(repo_id=str(dataset_path),
                                 episodes=[episode_idx])
        episode_data = dataset.hf_dataset.filter(
            lambda x: x["episode_index"] == episode_idx
        )
        actions = episode_data.select_columns("action")
        num_frames = len(episode_data)

        print(f"  Frames: {num_frames}, FPS: {dataset.fps}")
        print("  Replaying...\n")

        for idx in range(num_frames):
            t0 = time.perf_counter()
            action = {
                name: float(actions[idx]["action"][i])
                for i, name in enumerate(dataset.features["action"]["names"])
            }
            robot.send_action(action)

            if (idx + 1) % 10 == 0:
                print(f"    Frame {idx + 1}/{num_frames}")

            busy_wait(1.0 / dataset.fps - (time.perf_counter() - t0))

        print("  Replay completed!")
        return True

    except Exception as e:
        print(f"  Error during replay: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        if robot is not None:
            try:
                robot.disconnect()
                print("  Robot disconnected")
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main Segregator (Replay Approach)
# ---------------------------------------------------------------------------

COLOR_TO_BIN = {
    "red":   "record_bin1",
    "blue":  "record_bin2",
    "white": "record_discard",
}

COLOR_LABEL = {
    "red":   "Bin 1 (Red)",
    "blue":  "Bin 2 (Blue)",
    "white": "Discard (White)",
}


class ItemSegregatorReplay:
    """
    Detects item color via vision, then replays the appropriate pre-recorded
    episode to pick the item and place it in the correct bin.
    """

    def __init__(self, base_path: str, port: str = "/dev/ttyACM1",
                 camera_id: int = 0, episodes_per_bin: int = 4):
        self.base_path = base_path
        self.port = port

        if not Path(base_path).exists():
            raise ValueError(f"Base path does not exist: {base_path}")

        # Validate that dataset folders exist
        for folder in COLOR_TO_BIN.values():
            p = Path(base_path) / folder
            if not p.exists():
                print(f"  Warning: dataset folder missing: {p}")

        # Episode queues – one per bin folder, cycling through indices
        self.episode_queues: dict[str, deque] = {
            folder: deque(range(episodes_per_bin))
            for folder in COLOR_TO_BIN.values()
        }

        # History log
        self.history: list[dict] = []

        # Vision
        self.detector = ItemColorDetector(camera_id)

    def calibrate(self) -> bool:
        return self.detector.calibrate_pickup_zone()

    def _next_episode(self, folder: str) -> int:
        """Get the next episode index for a folder, cycling if needed."""
        q = self.episode_queues[folder]
        if not q:
            # Refill (cycle)
            q.extend(range(4))
        return q.popleft()

    def segregate_one(self) -> bool:
        """
        Handle one item: detect color -> replay episode -> log result.

        Returns True if an item was processed, False if detection failed.
        """
        print("\nPlace an item in the pickup zone and press ENTER to detect...")
        input()

        print("Detecting item color...")
        color = self.detector.detect_color_stable(required_frames=10,
                                                   timeout=15)

        if color is None:
            print("No item detected. Make sure the item is in the pickup zone.")
            return False

        folder = COLOR_TO_BIN[color]
        label = COLOR_LABEL[color]
        episode_idx = self._next_episode(folder)

        print(f"\n  Detected color : {color.upper()}")
        print(f"  Destination    : {label}")
        print(f"  Dataset folder : {folder}")
        print(f"  Episode index  : {episode_idx}")
        print("  Starting robot movement...")

        success = replay_episode(self.base_path, folder, episode_idx, self.port)

        self.history.append({
            "color": color,
            "destination": label,
            "folder": folder,
            "episode": episode_idx,
            "success": success,
        })

        if success:
            print(f"  Item ({color}) sent to {label} successfully!")
        else:
            print(f"  Robot replay failed for {label}, but continuing...")

        return True

    def run(self):
        """Main loop: repeatedly segregate items until user quits."""
        print("=" * 60)
        print("  ITEM SEGREGATOR  -  Replay Approach")
        print("=" * 60)
        print(f"  Red   -> Bin 1")
        print(f"  Blue  -> Bin 2")
        print(f"  White -> Discard")
        print("=" * 60)

        if not self.calibrate():
            print("Calibration failed. Exiting.")
            return

        item_count = 0
        while True:
            self.detector.show_live()
            self.segregate_one()
            item_count += 1

            cont = input("\nProcess another item? (yes/no): ").strip().lower()
            if cont not in ("yes", "y"):
                break

        self._print_summary()
        self.detector.release()

    def _print_summary(self):
        print("\n" + "=" * 60)
        print("  SESSION SUMMARY")
        print("=" * 60)
        for i, entry in enumerate(self.history, 1):
            status = "OK" if entry["success"] else "FAIL"
            print(f"  {i}. {entry['color'].upper():6s} -> {entry['destination']}"
                  f"  (ep {entry['episode']}) [{status}]")
        print("=" * 60 + "\n")


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("  Item Segregator  -  Replay Approach")
    print("=" * 60)

    base_path = input(
        "Enter base dataset path (default: /home/taksh/lerobot_datasets): "
    ).strip()
    if not base_path:
        base_path = "/home/taksh/lerobot_datasets"

    if not Path(base_path).exists():
        print(f"Error: Base path does not exist: {base_path}")
        return

    port = input(
        "Enter robot serial port (default: /dev/ttyACM1): "
    ).strip()
    if not port:
        port = "/dev/ttyACM1"

    camera_input = input("Enter camera ID (default: 0): ").strip()
    camera_id = 0
    if camera_input:
        try:
            camera_id = int(camera_input)
        except ValueError:
            print("Invalid camera ID, using default (0)")

    episodes_input = input(
        "Episodes per bin folder (default: 4): "
    ).strip()
    episodes_per_bin = 4
    if episodes_input:
        try:
            episodes_per_bin = int(episodes_input)
        except ValueError:
            print("Invalid number, using default (4)")

    try:
        seg = ItemSegregatorReplay(base_path, port, camera_id, episodes_per_bin)
        seg.run()
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
