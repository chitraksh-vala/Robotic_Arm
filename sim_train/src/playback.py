"""
Motion Playback Utility
Play back recorded motions with visualization
"""

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import time

try:
    from .recorder import MotionRecorder, MotionPlayer
except ImportError:
    from recorder import MotionRecorder, MotionPlayer


class PlaybackSimulator:
    """Simulator for playing back recorded motions"""
    
    def __init__(self, model_path: str = "mujoco_model/so_arm.xml"):
        """Initialize playback simulator"""
        self.model_path = Path(model_path)
        
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        
        self.player: MotionPlayer = None
        self.running = True
    
    def load_motion(self, filepath: str) -> bool:
        """Load a recorded motion"""
        recorder = MotionRecorder()
        data = recorder.load_recording(filepath)
        
        if not data:
            print(f"Failed to load motion: {filepath}")
            return False
        
        self.player = MotionPlayer(data)
        print(f"Loaded motion: {filepath}")
        return True
    
    def step(self):
        """Advance simulation by one step"""
        if self.player and self.player.is_playing:
            # Get target positions
            target_positions = self.player.get_target_positions()
            
            # Set joint positions (simplified playback)
            # In a real scenario, you'd use controllers to reach target positions
            self.data.qpos[:len(target_positions)] = target_positions
            
            # Advance playback
            self.player.advance_frame(self.model.opt.timestep)
        
        mujoco.mj_step(self.model, self.data)
    
    def get_status_string(self) -> str:
        """Get status string"""
        if not self.player:
            return "No motion loaded"
        
        status = []
        status.append(f"Playing: {self.player.is_playing}")
        status.append(f"Progress: {self.player.get_progress()*100:.1f}%")
        status.append(f"Frame: {self.player.current_frame}/{len(self.player.joint_positions)}")
        status.append(f"Duration: {self.player.get_duration():.2f}s")
        
        return " | ".join(status)
    
    def run(self, filepath: str):
        """Run playback"""
        if not self.load_motion(filepath):
            return
        
        print("\nPlayback Controls:")
        print("  SPACE: Play/Pause")
        print("  R: Reset")
        print("  ESC: Exit")
        print()
        
        self.player.start_playback()
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
            
            paused = False
            
            while viewer.is_running() and self.running:
                # Handle keyboard input
                if viewer.event_type == mujoco.mjtEvent.mjEVENT_KEYDOWN:
                    if viewer.event.key == mujoco.mjtKey.mjKEY_SPACE:
                        if self.player.is_playing:
                            paused = True
                            self.player.is_playing = False
                        elif paused:
                            self.player.is_playing = True
                            paused = False
                    elif viewer.event.key == mujoco.mjtKey.mjKEY_R:
                        self.player.start_playback()
                    elif viewer.event.key == mujoco.mjtKey.mjKEY_ESCAPE:
                        break
                
                # Step simulation
                self.step()
                
                # Update viewer
                with viewer.lock():
                    viewer.opt.label = self.get_status_string()
                
                viewer.sync()
                
                # Check if playback finished
                if self.player and not self.player.is_playing and self.player.current_frame >= len(self.player.joint_positions):
                    print("Playback finished")
                    break


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Motion Playback Utility")
    parser.add_argument("motion_file", nargs="?", help="Motion file to play (HDF5 format)")
    parser.add_argument("--list", action="store_true", help="List available recordings")
    
    args = parser.parse_args()
    
    if args.list:
        recorder = MotionRecorder()
        recordings = recorder.list_recordings()
        if recordings:
            print("\nAvailable Recordings:")
            for i, rec in enumerate(recordings, 1):
                info = recorder.get_recording_info(rec)
                print(f"{i}. {info['filename']} ({info['duration']:.2f}s, {info['num_frames']} frames)")
        else:
            print("No recordings found")
        return
    
    if not args.motion_file:
        # Show usage
        recorder = MotionRecorder()
        recordings = recorder.list_recordings()
        
        print("Motion Playback Utility")
        print()
        if recordings:
            print("Available recordings:")
            for i, rec in enumerate(recordings, 1):
                print(f"  {i}. {rec}")
            print()
            print("Usage: python playback.py <motion_file>")
            print("Example: python playback.py recordings/motion_20260522_143022.h5")
        else:
            print("No recordings found. Record a motion first using simulator.py")
        return
    
    # Play back the motion
    sim = PlaybackSimulator()
    sim.run(args.motion_file)


if __name__ == "__main__":
    main()
