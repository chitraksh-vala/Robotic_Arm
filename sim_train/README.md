# SO Arm MuJoCo Simulator

MuJoCo physics simulation for the SO robotic arm with DualSense controller support for motion recording and training data collection.

## Features

- **MuJoCo Physics Simulation**: Accurate physics-based simulation of the SO arm
- **DualSense Controller Support**: Real-time control using PlayStation 5 DualSense controller
- **Motion Recording**: Record trajectories for training and analysis
- **Motion Playback**: Play back recorded trajectories with visualization
- **Interactive Visualization**: Real-time 3D visualization with MuJoCo viewer
- **Motion Analysis**: Compare, analyze, and extract statistics from recordings

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Verify installation
python validate.py

# 3. Run simulator
cd src
python simulator.py
```

## Setup

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

This installs all required packages including pygame with pre-built SDL2 support.

### 2. Connect DualSense Controller

Connect your DualSense controller via USB or Bluetooth before running the simulator.

## Controller Mapping

| Input | Action |
|-------|--------|
| **Left Stick X/Y** | Shoulder Pan / Lift |
| **Right Stick X/Y** | Wrist Roll / Elbow |
| **L2 Trigger** | Wrist Pitch Up |
| **R2 Trigger** | Wrist Pitch Down |
| **L1 Button** | Gripper Close |
| **R1 Button** | Gripper Open |
| **△ Triangle** | **Toggle Recording** |
| **✕ Cross** | Reset to Home |

## Commands

```bash
# Interactive simulator
python simulator.py

# Headless (no viewer)
python simulator.py --headless --duration 60

# Without controller
python simulator.py --no-controller

# List recordings
python motion_utils.py list

# Analyze recording
python motion_utils.py analyze recordings/motion_*.h5

# Compare recordings
python motion_utils.py compare recordings/motion1.h5 recordings/motion2.h5

# Playback recording
python playback.py recordings/motion_*.h5

# Training template
python training_example.py
```

## Motion Recording Workflow

1. **Start simulator**: `python simulator.py`
2. **Warm up arm**: Move all joints slowly (~30 seconds)
3. **Record motion**: Press **Triangle** button
4. **Perform motion**: Move arm for 5-30 seconds
5. **Stop recording**: Press **Triangle** again
6. **File saved**: `recordings/motion_YYYYMMDD_HHMMSS.h5`
7. **Analyze**: `python motion_utils.py analyze recordings/motion_*.h5`
8. **Playback**: `python playback.py recordings/motion_*.h5`

## Project Structure

```
sim_train/
├── mujoco_model/
│   └── so_arm.xml           # 6-DOF arm + 2-DOF gripper model
├── src/
│   ├── simulator.py         # Main simulator
│   ├── controller.py        # DualSense interface
│   ├── recorder.py          # Recording system
│   ├── playback.py          # Playback player
│   ├── motion_utils.py      # Motion analysis
│   └── training_example.py  # Training template
├── recordings/              # Motion storage
├── config/simulator_config.yaml
├── requirements.txt
└── validate.py
```

## Recording Format

HDF5 files containing:
- `joint_positions`: Shape (N_frames, 8)
- `joint_velocities`: Shape (N_frames, 8)
- `commands`: Shape (N_frames, 8)
- `timestamps`: Shape (N_frames,)

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Controller not found | Connect DualSense via USB, wait 5 sec, retry |
| ModuleNotFoundError | `pip install -r requirements.txt` |
| Model file not found | Ensure you `cd src` before running |
| Simulator won't open | Try `python simulator.py --no-controller` |
| No recordings saved | Run `python validate.py` to check setup |

### DualSense Not Detected

- Check that the controller is connected via USB or Bluetooth
- Verify SDL2 is properly installed
- Try running: `pygame.joystick.get_count()` to check if pygame detects the controller
- Restart the simulator after connecting the controller

### MuJoCo Model Issues

- Ensure the XML file path is correct: `mujoco_model/so_arm.xml`
- Check that MuJoCo is properly installed: `python -c "import mujoco; print(mujoco.__version__)"`
- Verify all required dependencies are installed with `python validate.py`

### Playback Issues

- Verify the recording file exists and is a valid HDF5 file
- Use `motion_utils.py analyze` to check recording integrity
- Ensure you're in the `src` directory when running playback
- Check that timestamps are valid in the recording file

## Tips

- **Smooth movements**: Avoid jerky motions when recording
- **Multiple takes**: Record same motion 3-5 times for consistency
- **Headless training**: Use `--headless` flag for faster data collection
- **Warm up**: Always move arm 30 seconds before important recordings

## Training Data

Use `training_example.py` to:
- Load all recordings
- Extract (state, action) pairs
- Normalize for neural networks
- Get dataset statistics

Example PyTorch policy network:

```python
import torch.nn as nn

class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 64), nn.ReLU(),
            nn.Linear(64, 64), nn.ReLU(),
            nn.Linear(64, action_dim), nn.Tanh()
        )
    
    def forward(self, state):
        return self.net(state)
```

## Next Steps

1. **Calibrate the model**: Adjust inertia, friction, and mass parameters to match real arm
2. **Import CAD models**: Convert `.part` files to MuJoCo-compatible mesh format
3. **Develop training pipeline**: Use recorded trajectories for imitation learning
4. **Add sensors**: Implement force/torque sensors, cameras for vision-based learning
5. **Test on hardware**: Export trained policy and validate on real SO arm

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [DualSense Documentation](https://github.com/joehoyle/DualSense)
- [Pygame Joystick API](https://www.pygame.org/docs/ref/joystick.html)
- [H5py Documentation](https://www.h5py.org/)

## License

This project is provided for educational and research purposes.
