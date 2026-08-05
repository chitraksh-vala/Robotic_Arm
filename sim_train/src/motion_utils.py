"""
Motion Management Utility
Tools for managing, analyzing, and playing back recorded motions
"""

import sys
from pathlib import Path
import numpy as np

try:
    from .recorder import MotionRecorder, MotionPlayer
except ImportError:
    from recorder import MotionRecorder, MotionPlayer


def list_motions():
    """List all recorded motions"""
    recorder = MotionRecorder()
    recordings = recorder.list_recordings()
    
    if not recordings:
        print("No recordings found")
        return
    
    print("\nAvailable Recordings:")
    print("-" * 80)
    for i, recording in enumerate(recordings, 1):
        info = recorder.get_recording_info(recording)
        print(f"{i}. {info['filename']}")
        print(f"   Duration: {info['duration']:.2f}s")
        print(f"   Frames: {info['num_frames']}")
        print(f"   Joints: {info['num_joints']}")
        print()


def analyze_motion(filepath: str):
    """Analyze a recorded motion"""
    recorder = MotionRecorder()
    data = recorder.load_recording(filepath)
    
    if not data:
        print(f"Failed to load {filepath}")
        return
    
    positions = data['joint_positions']
    velocities = data['joint_velocities']
    timestamps = data['timestamps']
    
    print(f"\nMotion Analysis: {Path(filepath).name}")
    print("-" * 80)
    print(f"Duration: {timestamps[-1]:.2f}s")
    print(f"Frames: {len(positions)}")
    print(f"Joints: {positions.shape[1]}")
    print(f"Framerate: {len(positions) / timestamps[-1]:.1f} Hz")
    print()
    
    print("Joint Statistics:")
    print("-" * 80)
    for j in range(positions.shape[1]):
        pos_min = np.min(positions[:, j])
        pos_max = np.max(positions[:, j])
        pos_range = pos_max - pos_min
        vel_max = np.max(np.abs(velocities[:, j]))
        
        print(f"Joint {j}:")
        print(f"  Position range: [{pos_min:.3f}, {pos_max:.3f}] (Δ={pos_range:.3f})")
        print(f"  Max velocity: {vel_max:.3f}")


def compare_motions(filepath1: str, filepath2: str):
    """Compare two recorded motions"""
    recorder = MotionRecorder()
    data1 = recorder.load_recording(filepath1)
    data2 = recorder.load_recording(filepath2)
    
    if not data1 or not data2:
        print("Failed to load motion files")
        return
    
    pos1 = data1['joint_positions']
    pos2 = data2['joint_positions']
    
    print(f"\nMotion Comparison")
    print("-" * 80)
    print(f"Motion 1: {Path(filepath1).name}")
    print(f"Motion 2: {Path(filepath2).name}")
    print()
    
    # Align recordings by length
    min_len = min(len(pos1), len(pos2))
    pos1 = pos1[:min_len]
    pos2 = pos2[:min_len]
    
    # Calculate differences
    diff = np.abs(pos1 - pos2)
    mean_diff = np.mean(diff, axis=0)
    max_diff = np.max(diff, axis=0)
    
    print("Joint Differences (RMSE):")
    for j in range(pos1.shape[1]):
        rmse = np.sqrt(np.mean(diff[:, j] ** 2))
        print(f"  Joint {j}: {rmse:.4f} (max: {max_diff[j]:.4f})")


def main():
    """Main entry point"""
    if len(sys.argv) < 2:
        print("Motion Management Utility")
        print()
        print("Usage:")
        print("  python motion_utils.py list                    - List all recordings")
        print("  python motion_utils.py analyze <file>          - Analyze a recording")
        print("  python motion_utils.py compare <file1> <file2> - Compare two recordings")
        return
    
    command = sys.argv[1].lower()
    
    if command == "list":
        list_motions()
    elif command == "analyze":
        if len(sys.argv) < 3:
            print("Please specify a recording file")
            return
        analyze_motion(sys.argv[2])
    elif command == "compare":
        if len(sys.argv) < 4:
            print("Please specify two recording files")
            return
        compare_motions(sys.argv[2], sys.argv[3])
    else:
        print(f"Unknown command: {command}")


if __name__ == "__main__":
    main()
