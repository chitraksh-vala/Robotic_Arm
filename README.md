# Robotic Arm Workspace

This repository contains several SO-ARM101 robotic-arm workflows:

- browser-based simulation and hardware control
- physical hardware bridge control
- reinforcement-learning pick-and-place training
- CV item segregation
- CV tic tac toe
- MuJoCo motion recording and playback
- dual-arm experiments

The project is a workspace, not one single app. Most folders can be run independently, so start from the section that matches the function you want to use.

## Repository Map

| Folder | Purpose |
|---|---|
| `sim2hw/` | Browser dashboard, MuJoCo IK server, and optional hardware bridge for the real arm |
| `RL_train/` | Gymnasium/MuJoCo RL environment, SAC training script, policy viewer, and related assets |
| `CV-seg/` | Computer-vision item segregation and tic tac toe scripts |
| `sim_train/` | MuJoCo simulator, trajectory recording, trajectory playback, and recording utilities |
| `dual_arm/` | Separate dual-arm simulation/dashboard experiments |

## Basic Setup

Use a Python environment with the needed robotics packages installed. The exact packages depend on which workflow you run.

For the MuJoCo simulation, recording, and playback tools:

```powershell
cd sim_train
pip install -r requirements.txt
python validate.py
```

For hardware/CV workflows, you also need the LeRobot SO101/Feetech stack and OpenCV available in the environment. The scripts expect a connected SO101 follower arm and a camera.

Common serial-port examples:

```text
Windows: COM4
Linux:   /dev/ttyUSB0 or /dev/ttyACM1
```

You can usually find the arm port with:

```powershell
lerobot-find-port
```

This README assumes the LeRobot `Scripts` folder is already on your Windows `PATH`, so LeRobot commands can be run by name.

## LeRobot Calibration

Run this before using the physical arm through `sim2hw/hw_bridge.py`, the CV scripts, or any future leader-follower control. Calibration saves the motor offsets/ranges that LeRobot uses when reading and commanding joint positions.

Find the port first:

```powershell
lerobot-find-port
```

Calibrate the follower arm, which is the arm being controlled:

```powershell
lerobot-calibrate --robot.type=so101_follower --robot.port=COM4 --robot.id=follower
```

Replace `COM4` with your actual follower-arm port. This repository's `sim2hw/hw_bridge.py` currently loads the default calibration id `follower`, so using `--robot.id=follower` keeps it aligned with the existing bridge code.

If you are using another SO101 arm as the physical external controller, calibrate that leader arm too:

```powershell
lerobot-calibrate --teleop.type=so101_leader --teleop.port=COM5 --teleop.id=leader
```

Replace `COM5` with the leader-arm port. During calibration, place the arm in its centered pose when prompted, press Enter, then move each joint through its full range of motion.

For a brand-new arm whose motor IDs have not been configured yet, run motor setup before calibration:

```powershell
lerobot-setup-motors --robot.type=so101_follower --robot.port=COM4
```

For a brand-new leader arm:

```powershell
lerobot-setup-motors --teleop.type=so101_leader --teleop.port=COM5
```

## Browser Control

Use this when you want to control the arm from the web dashboard. This can run in simulation only, or mirror commands to real hardware if the hardware bridge is also running.

Start the IK server:

```powershell
cd sim2hw
python ik_server.py --viewer
```

Headless mode is also supported:

```powershell
cd sim2hw
python ik_server.py
```

The IK server runs on:

```text
http://127.0.0.1:7842
```

Start the dashboard file server in a second terminal:

```powershell
cd sim2hw
python -m http.server 8000
```

Open:

```text
http://127.0.0.1:8000/so101_dashboard.html
```

Optional real-hardware bridge, in a third terminal:

```powershell
cd sim2hw
python hw_bridge.py --port COM4
```

Replace `COM4` with your actual arm port. The hardware bridge runs on:

```text
http://127.0.0.1:7843
```

Useful files:

| File | Use |
|---|---|
| `sim2hw/ik_server.py` | Backend server for IK, MuJoCo simulation, movement commands, gripper commands, and pipelines |
| `sim2hw/so101_dashboard.html` | Browser dashboard UI |
| `sim2hw/hw_bridge.py` | Sends commands to the physical SO101/Feetech hardware |
| `sim2hw/scene_full.xml` | MuJoCo scene used by the server |
| `sim2hw/assets/` | Meshes required by the scene |

## Hardware Bridge Only

Use this when another program needs to send HTTP commands directly to the physical arm without opening the browser dashboard.

```powershell
cd sim2hw
python hw_bridge.py --port COM4
```

Then send commands to:

```text
http://127.0.0.1:7843
```

This is the part you would build on for a future leader-follower setup, where one robotic arm acts as the controller and the other arm copies it.

## Leader-Follower Teleop Control

Use this when you want to control the follower arm by physically moving a second SO101 leader arm. This is LeRobot's direct arm-to-arm teleoperation mode.

Before starting teleop:

- calibrate the follower arm with `--robot.id=follower`
- calibrate the leader arm with `--teleop.id=leader`
- confirm both COM ports with `lerobot-find-port`
- make sure no other script is using either port

Start teleop:

```powershell
lerobot-teleoperate --robot.type=so101_follower --robot.port=COM6 --robot.id=follower --teleop.type=so101_leader --teleop.port=COM7 --teleop.id=leader
```

Replace:

| Value | Meaning |
|---|---|
| `COM6` | follower arm port, the arm that moves |
| `COM7` | leader arm port, the arm you move by hand |
| `follower` | calibration id used for the follower arm |
| `leader` | calibration id used for the leader arm |

Optional live display:

```powershell
lerobot-teleoperate --robot.type=so101_follower --robot.port=COM6 --robot.id=follower --teleop.type=so101_leader --teleop.port=COM7 --teleop.id=leader --display_data=true
```

To stop teleop, press `Ctrl+C` in the terminal. The script disconnects both arms during shutdown.

## Motion Recording

The current recording workflow lives in `sim_train`. It records joint positions, joint velocities, command values, and timestamps into HDF5 files.

Run the simulator:

```powershell
cd sim_train
python src\simulator.py
```

Run without controller input:

```powershell
cd sim_train
python src\simulator.py --no-controller
```

Run headless for a fixed duration:

```powershell
cd sim_train
python src\simulator.py --headless --duration 60
```

Recording files are saved as:

```text
sim_train/recordings/motion_YYYYMMDD_HHMMSS.h5
```

Important files:

| File | Use |
|---|---|
| `sim_train/src/simulator.py` | Runs MuJoCo simulation and contains the recording loop |
| `sim_train/src/recorder.py` | Saves and loads recorded trajectories |
| `sim_train/mujoco_model/so_arm.xml` | Single-arm MuJoCo model |
| `sim_train/assets/` | Meshes required by the model |

Note: `sim_train/src/controller.py` is the DualSense/joystick input module. If you are not using joystick control, treat it as the controller-specific part.

## Motion Playback

List available recordings:

```powershell
cd sim_train
python src\playback.py --list
```

Play one recording:

```powershell
cd sim_train
python src\playback.py recordings\motion_YYYYMMDD_HHMMSS.h5
```

Analyze recordings:

```powershell
cd sim_train
python src\motion_utils.py list
python src\motion_utils.py analyze recordings\motion_YYYYMMDD_HHMMSS.h5
```

Compare two recordings:

```powershell
cd sim_train
python src\motion_utils.py compare recordings\motion_1.h5 recordings\motion_2.h5
```

Important files:

| File | Use |
|---|---|
| `sim_train/src/playback.py` | Visual playback of recorded HDF5 trajectories |
| `sim_train/src/motion_utils.py` | Lists, analyzes, and compares recordings |
| `sim_train/src/recorder.py` | Loads the saved recording data |

## CV Item Segregator

There are two item-segregation modes.

Realtime mode directly detects object color and sends hardcoded waypoint movements to the arm:

```powershell
cd CV-seg
python item_segregator_realtime.py
```

Replay mode detects object color and then replays a pre-recorded LeRobot episode for the matching bin:

```powershell
cd CV-seg
python item_segregator_replay.py
```

The scripts ask for the serial port, camera ID, and dataset path when needed.

Expected replay dataset folders:

```text
record_bin1/
record_bin2/
record_discard/
```

Important files:

| File | Use |
|---|---|
| `CV-seg/item_segregator_realtime.py` | Detects red/blue/white items and moves the arm with built-in waypoints |
| `CV-seg/item_segregator_replay.py` | Detects red/blue/white items and replays the matching recorded dataset |

## Tic Tac Toe

Run the robot tic tac toe workflow:

```powershell
cd CV-seg
python cv_tictactoe.py
```

The script prompts for:

- dataset base path
- robot serial port
- camera ID
- whether to use computer vision

Expected replay dataset folders:

```text
record1/
record2/
record3/
record4/
record5/
record6/
record7/
record8/
record9/
recordparty/
```

Important file:

| File | Use |
|---|---|
| `CV-seg/cv_tictactoe.py` | Detects board state, chooses robot moves using minimax, and replays the matching movement dataset |

## Reinforcement Learning

The RL project trains a SAC policy for MuJoCo pick-and-place.

Train with defaults:

```powershell
cd RL_train
python train_pickplace.py
```

Train for a custom number of timesteps:

```powershell
cd RL_train
python train_pickplace.py --timesteps 1000000
```

Use fewer environments for debugging:

```powershell
cd RL_train
python train_pickplace.py --n_envs 1
```

Render while training, slower but useful for inspection:

```powershell
cd RL_train
python train_pickplace.py --render
```

Resume from a checkpoint:

```powershell
cd RL_train
python train_pickplace.py --checkpoint checkpoints\sac_pickplace_50000_steps
```

Evaluate an existing model:

```powershell
cd RL_train
python train_pickplace.py --eval_only checkpoints\best_model --render
```

View a trained policy:

```powershell
cd RL_train
python view_policy.py
```

View a specific model:

```powershell
cd RL_train
python view_policy.py --model checkpoints\best_model --episodes 20
```

Run a random baseline:

```powershell
cd RL_train
python view_policy.py --random
```

Important files:

| File | Use |
|---|---|
| `RL_train/so101_env.py` | Gymnasium MuJoCo environment |
| `RL_train/train_pickplace.py` | SAC training and evaluation entry point |
| `RL_train/view_policy.py` | Interactive MuJoCo viewer for trained/random policies |
| `RL_train/scene_full.xml` | Main MuJoCo scene used by the environment |
| `RL_train/assets/` | Meshes required by the scene |

Outputs are written to:

```text
RL_train/checkpoints/
RL_train/logs/
```

## Dual Arm Experiments

This is a separate experimental area for controlling two simulated SO101 arms.

Start the dual-arm IK server:

```powershell
cd dual_arm
python dual_ik_server.py --viewer
```

Start the dashboard:

```powershell
cd dual_arm
python -m http.server 8001
```

Open:

```text
http://127.0.0.1:8001/dual_dashboard.html
```

Important files:

| File | Use |
|---|---|
| `dual_arm/dual_ik_server.py` | Dual-arm IK backend |
| `dual_arm/dual_dashboard.html` | Browser dashboard for two arms |
| `dual_arm/scene_dual.xml` | MuJoCo scene with two arms |
| `dual_arm/test_*.py` | Diagnostic tests for dual-arm behavior |

## Quick Troubleshooting

| Problem | Try this |
|---|---|
| MuJoCo model does not load | Run from the folder shown in the command examples so relative XML paths resolve correctly |
| Dashboard cannot connect | Make sure `ik_server.py` is running on port `7842` |
| Hardware does not move | Check the serial port and start `hw_bridge.py` on port `7843` |
| `lerobot-teleoperate` is not recognized | Add the Python `Scripts` folder containing the LeRobot `.exe` files to Windows `PATH`, then open a new PowerShell |
| Teleop cannot find motors | Check arm power, daisy-chain cables, motor IDs, and that the correct COM ports are used for follower and leader |
| Camera does not open | Try a different camera ID, usually `0`, `1`, or `2` |
| Recording file not found | Run playback from `sim_train` and check `recordings/` |
| RL import error | Install `gymnasium`, `stable-baselines3`, `mujoco`, and other RL dependencies in the active Python environment |

## Notes

- Run each workflow from its own folder unless a command says otherwise.
- The browser controller is in `sim2hw`.
- The DualSense/joystick-specific controller is `sim_train/src/controller.py`.
- Direct leader-follower arm control is handled by LeRobot's `lerobot-teleoperate` command.
