"""
Example Training Setup
Shows how to use recorded motions for policy training
"""

import numpy as np
from typing import Tuple, List

try:
    from .recorder import MotionRecorder
except ImportError:
    from recorder import MotionRecorder


class TrainingDataset:
    """Dataset for training policies from recorded motions"""
    
    def __init__(self, recording_dir: str = "recordings"):
        """Initialize training dataset"""
        self.recording_dir = Path(recording_dir)
        self.motions = []
        self.metadata = []
        self.load_all_motions()
    
    def load_all_motions(self):
        """Load all available recordings"""
        recorder = MotionRecorder()
        recordings = recorder.list_recordings()
        
        for recording_path in recordings:
            data = recorder.load_recording(recording_path)
            if data:
                self.motions.append(data)
                self.metadata.append(recorder.get_recording_info(recording_path))
        
        print(f"Loaded {len(self.motions)} motion(s)")
    
    def get_motion(self, index: int) -> dict:
        """Get a single motion"""
        if 0 <= index < len(self.motions):
            return self.motions[index]
        return None
    
    def get_trajectory_pairs(self, window_size: int = 10) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert motions to (state, action) pairs for imitation learning
        
        Args:
            window_size: Number of frames to use as context
        
        Returns:
            Tuple of (states, actions) arrays
        """
        states = []
        actions = []
        
        for motion in self.motions:
            positions = motion['joint_positions']
            velocities = motion['joint_velocities']
            commands = motion['commands']
            
            # Create state: [position, velocity]
            state_data = np.hstack([positions, velocities])
            
            # Action: control commands
            action_data = commands
            
            # Create sliding window of trajectory pairs
            for i in range(len(state_data) - 1):
                states.append(state_data[i])
                actions.append(action_data[i])
        
        return np.array(states), np.array(actions)
    
    def get_statistics(self) -> dict:
        """Get dataset statistics"""
        if not self.motions:
            return {}
        
        total_frames = sum(len(m['joint_positions']) for m in self.motions)
        total_duration = sum(m['timestamps'][-1] for m in self.motions)
        
        # Collect all positions for normalization stats
        all_positions = np.vstack([m['joint_positions'] for m in self.motions])
        
        stats = {
            'num_motions': len(self.motions),
            'total_frames': total_frames,
            'total_duration': total_duration,
            'avg_motion_duration': total_duration / len(self.motions),
            'position_mean': np.mean(all_positions, axis=0),
            'position_std': np.std(all_positions, axis=0),
            'position_min': np.min(all_positions, axis=0),
            'position_max': np.max(all_positions, axis=0),
        }
        
        return stats
    
    def normalize_states(self, states: np.ndarray, stats: dict) -> np.ndarray:
        """Normalize states using dataset statistics"""
        normalized = states.copy()
        mean = stats['position_mean']
        std = stats['position_std']
        
        # Avoid division by zero
        std[std == 0] = 1.0
        
        normalized[:, :len(mean)] = (normalized[:, :len(mean)] - mean) / std
        
        return normalized


class ImitationLearningExample:
    """Example of how to train a policy using recorded motions"""
    
    @staticmethod
    def train_simple_policy():
        """
        Example: Train a simple neural network policy
        
        This is a skeleton - implement with your preferred framework:
        - TensorFlow
        - PyTorch
        - JAX
        """
        
        # Load dataset
        dataset = TrainingDataset()
        
        if not dataset.motions:
            print("No motions found. Record some motions first!")
            return
        
        # Get statistics
        stats = dataset.get_statistics()
        print("\nDataset Statistics:")
        print(f"  Motions: {stats['num_motions']}")
        print(f"  Total frames: {stats['total_frames']}")
        print(f"  Total duration: {stats['total_duration']:.2f}s")
        print(f"  Avg motion duration: {stats['avg_motion_duration']:.2f}s")
        
        # Get training data
        states, actions = dataset.get_trajectory_pairs()
        print(f"\nTraining Data:")
        print(f"  State shape: {states.shape} (samples, features)")
        print(f"  Action shape: {actions.shape} (samples, actions)")
        
        # Normalize states
        normalized_states = dataset.normalize_states(states, stats)
        print(f"  Normalized state range: [{normalized_states.min():.3f}, {normalized_states.max():.3f}]")
        
        print("\n✓ Data ready for training!")
        print("\nNext steps:")
        print("  1. Split data into train/validation sets")
        print("  2. Create neural network with input=state, output=action")
        print("  3. Train with MSE loss to imitate recorded motions")
        print("  4. Validate policy in simulator")
        
        # Example structure for PyTorch:
        print("\nPyTorch example structure:")
        print("""
import torch
import torch.nn as nn

# Create simple policy network
class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim),
            nn.Tanh()
        )
    
    def forward(self, state):
        return self.net(state)

# Create model
state_dim = states.shape[1]
action_dim = actions.shape[1]
policy = PolicyNetwork(state_dim, action_dim)

# Training loop
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)
for epoch in range(100):
    # Forward pass
    predicted = policy(torch.from_numpy(normalized_states))
    # Compute loss
    loss = nn.MSELoss()(predicted, torch.from_numpy(actions))
    # Backward pass
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
        """)
    
    return dataset, stats


def main():
    """Run example training setup"""
    print("=" * 60)
    print("Imitation Learning Example")
    print("=" * 60)
    print()
    
    print("This script shows how to:")
    print("  1. Load recorded motions")
    print("  2. Extract (state, action) pairs for supervised learning")
    print("  3. Normalize data for neural networks")
    print("  4. Prepare data for policy training")
    print()
    
    try:
        dataset, stats = ImitationLearningExample.train_simple_policy()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
