"""
Motion Recording System
Records arm trajectories from controller input for later playback and training
"""

import h5py
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import List, Optional, Dict


class MotionRecorder:
    """Records and manages arm motion sequences"""
    
    def __init__(self, recordings_dir: str = "recordings"):
        """
        Initialize motion recorder
        
        Args:
            recordings_dir: Directory to store recordings
        """
        self.recordings_dir = Path(recordings_dir)
        self.recordings_dir.mkdir(exist_ok=True)
        
        self.is_recording = False
        self.joint_positions: List[np.ndarray] = []
        self.joint_velocities: List[np.ndarray] = []
        self.commands: List[np.ndarray] = []
        self.timestamps: List[float] = []
        self.start_time: Optional[float] = None
        self.metadata: Dict = {}
    
    def start_recording(self, metadata: Optional[Dict] = None):
        """
        Start recording a new trajectory
        
        Args:
            metadata: Optional metadata to store with recording
        """
        self.is_recording = True
        self.joint_positions = []
        self.joint_velocities = []
        self.commands = []
        self.timestamps = []
        self.start_time = None
        self.metadata = metadata or {}
        print("Recording started...")
    
    def stop_recording(self) -> str:
        """
        Stop recording and save to file
        
        Returns:
            Path to saved recording file
        """
        if not self.is_recording:
            print("Not currently recording")
            return ""
        
        self.is_recording = False
        
        if len(self.joint_positions) == 0:
            print("No data recorded")
            return ""
        
        filename = self._generate_filename()
        filepath = self.recordings_dir / filename
        
        self._save_recording(filepath)
        print(f"Recording saved to {filepath}")
        return str(filepath)
    
    def record_frame(self, 
                     time: float,
                     joint_positions: np.ndarray,
                     joint_velocities: np.ndarray,
                     commands: np.ndarray):
        """
        Record a single frame of motion data
        
        Args:
            time: Current simulation time
            joint_positions: Current joint positions
            joint_velocities: Current joint velocities
            commands: Current control commands
        """
        if not self.is_recording:
            return
        
        if self.start_time is None:
            self.start_time = time
        
        self.timestamps.append(time - self.start_time)
        self.joint_positions.append(joint_positions.copy())
        self.joint_velocities.append(joint_velocities.copy())
        self.commands.append(commands.copy())
    
    def _generate_filename(self) -> str:
        """Generate filename with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"motion_{timestamp}.h5"
    
    def _save_recording(self, filepath: Path):
        """Save recording to HDF5 file"""
        with h5py.File(filepath, 'w') as f:
            # Store metadata
            for key, value in self.metadata.items():
                if isinstance(value, (str, int, float, bool)):
                    f.attrs[key] = value
            
            # Store trajectory data
            positions_array = np.array(self.joint_positions)
            velocities_array = np.array(self.joint_velocities)
            commands_array = np.array(self.commands)
            timestamps_array = np.array(self.timestamps)
            
            f.create_dataset('joint_positions', data=positions_array, compression='gzip')
            f.create_dataset('joint_velocities', data=velocities_array, compression='gzip')
            f.create_dataset('commands', data=commands_array, compression='gzip')
            f.create_dataset('timestamps', data=timestamps_array, compression='gzip')
            
            f.attrs['num_frames'] = len(self.timestamps)
            f.attrs['duration'] = self.timestamps[-1] if self.timestamps else 0
    
    def load_recording(self, filepath: str) -> Dict:
        """
        Load a recorded trajectory
        
        Args:
            filepath: Path to recording file
        
        Returns:
            Dictionary containing trajectory data
        """
        filepath = Path(filepath)
        if not filepath.exists():
            print(f"Recording file not found: {filepath}")
            return {}
        
        data = {}
        with h5py.File(filepath, 'r') as f:
            # Load trajectory data
            data['joint_positions'] = f['joint_positions'][:]
            data['joint_velocities'] = f['joint_velocities'][:]
            data['commands'] = f['commands'][:]
            data['timestamps'] = f['timestamps'][:]
            
            # Load metadata
            data['metadata'] = dict(f.attrs)
        
        return data
    
    def list_recordings(self) -> List[str]:
        """List all available recordings"""
        recordings = list(self.recordings_dir.glob("motion_*.h5"))
        return sorted([str(r) for r in recordings])
    
    def get_recording_info(self, filepath: str) -> Dict:
        """Get information about a recording"""
        filepath = Path(filepath)
        if not filepath.exists():
            return {}
        
        info = {}
        with h5py.File(filepath, 'r') as f:
            info['filename'] = filepath.name
            info['num_frames'] = f.attrs['num_frames']
            info['duration'] = f.attrs['duration']
            info['num_joints'] = f['joint_positions'].shape[1]
            info['metadata'] = dict(f.attrs)
        
        return info


class MotionPlayer:
    """Plays back recorded motion sequences"""
    
    def __init__(self, recording_data: Dict):
        """
        Initialize motion player with loaded recording data
        
        Args:
            recording_data: Data from MotionRecorder.load_recording()
        """
        self.joint_positions = recording_data.get('joint_positions', np.array([]))
        self.joint_velocities = recording_data.get('joint_velocities', np.array([]))
        self.commands = recording_data.get('commands', np.array([]))
        self.timestamps = recording_data.get('timestamps', np.array([]))
        self.metadata = recording_data.get('metadata', {})
        
        self.current_frame = 0
        self.is_playing = False
        self.playback_speed = 1.0  # 1.0 = normal speed
    
    def start_playback(self):
        """Start playing back the motion"""
        self.current_frame = 0
        self.is_playing = True
    
    def stop_playback(self):
        """Stop playback"""
        self.is_playing = False
        self.current_frame = 0
    
    def get_target_positions(self) -> np.ndarray:
        """Get target positions for current frame"""
        if not self.is_playing or len(self.joint_positions) == 0:
            return np.zeros(self.joint_positions.shape[1] if len(self.joint_positions) > 0 else 8)
        
        if self.current_frame >= len(self.joint_positions):
            self.is_playing = False
            return self.joint_positions[-1]
        
        return self.joint_positions[self.current_frame]
    
    def advance_frame(self, dt: float):
        """Advance playback by time step"""
        if not self.is_playing:
            return
        
        # Find next frame based on playback speed
        if len(self.timestamps) > 1:
            frame_duration = self.timestamps[self.current_frame + 1] - self.timestamps[self.current_frame] if self.current_frame + 1 < len(self.timestamps) else 0.001
            self.current_frame += int(self.playback_speed * dt / frame_duration) if frame_duration > 0 else 1
        else:
            self.current_frame += 1
        
        if self.current_frame >= len(self.joint_positions):
            self.is_playing = False
    
    def get_duration(self) -> float:
        """Get total duration of the motion"""
        return float(self.timestamps[-1]) if len(self.timestamps) > 0 else 0.0
    
    def get_progress(self) -> float:
        """Get playback progress (0 to 1)"""
        if len(self.joint_positions) == 0:
            return 0.0
        return min(self.current_frame / len(self.joint_positions), 1.0)
