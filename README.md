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

## Quick Start: Vision Pick-and-Place

The camera detector finds objects on the workspace board, and the IK server and hardware bridge pick a selected one and place it at the spot set for its colour (any other colour goes to the default spot, Corner 2 unless you set one).

Set up once (Windows PowerShell, from this folder):

```powershell
.\setup.ps1             # creates .venv and installs requirements; -SimOnly skips the real-arm driver
```

Run (each part opens in its own window):

```powershell
.\run.ps1 -Port COM4    # IK server + hardware bridge + object detector
.\run.ps1               # simulation only, no real arm
```

If scripts are blocked: `powershell -ExecutionPolicy Bypass -File .\setup.ps1` (same for `run.ps1`). `Get-Help .\run.ps1 -Detailed` lists all options (camera, 3-D viewer, dashboard, extra arguments).

**New operator? Start with [PICK_AND_PLACE_README.md](PICK_AND_PLACE_README.md):** the full guide from setup and arm calibration to place spots, stacking and teaching.

| Part | Docs |
|---|---|
| Full operator guide (everything below in one place) | [PICK_AND_PLACE_README.md](PICK_AND_PLACE_README.md) |
| IK server and hardware bridge | [sim2hw/README.md](sim2hw/README.md) |
| Object detector: calibration, keys, teaching | [CV-seg/Robotic_Arm-main/README.md](CV-seg/Robotic_Arm-main/README.md) |
| Board-to-robot calibration points | [sim2hw/CV_IK_MAPPING.md](sim2hw/CV_IK_MAPPING.md) |

Requirements: Python 3.12 or 3.13 (3.10+ with `-SimOnly`), a USB camera, and for the real arm a calibrated SO-101 (first time: follow [First-Time Calibration](#first-time-calibration-so-arm101)). The packages are in `requirements.txt` and `requirements-hardware.txt`.

## Repository Map

| Folder | Purpose |
|---|---|
| `sim2hw/` | Browser dashboard, MuJoCo IK server, and optional hardware bridge for the real arm |
| `RL_train/` | Gymnasium/MuJoCo RL environment, SAC training script, policy viewer, and related assets |
| `CV-seg/` | Computer-vision item segregation and tic tac toe scripts; `CV-seg/Robotic_Arm-main/` holds the pick-and-place object detector |
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

## First-Time Calibration (SO-ARM101)

Do these steps in order the first time you use an arm with this project. Steps 1–3 are LeRobot's motor calibration, and every program here depends on them. Steps 4–9 calibrate the gripper, camera and board for vision pick-and-place.

Run `.\setup.ps1` first. The LeRobot commands below are installed in `.venv\Scripts`, so either activate the environment once per terminal:

```powershell
.\.venv\Scripts\Activate.ps1
```

or call them by full path, e.g. `.\.venv\Scripts\lerobot-find-port.exe`. Power the arm and connect its USB cable before starting.

### 1. Find the arm's serial port

```powershell
lerobot-find-port
```

Unplug the arm's USB cable when asked and press Enter. The port that disappeared (for example `COM4`) is the arm. Use it everywhere below.

### 2. Set the motor IDs (brand-new arm only)

Skip this if the arm has been used before. New STS3215 servos all have the same ID, so each one must be given its own:

```powershell
lerobot-setup-motors --robot.type=so101_follower --robot.port=COM4
```

When asked, connect **only that one motor** to the controller board and press Enter. It starts with `gripper`, then `wrist_roll`, `wrist_flex`, `elbow_flex`, `shoulder_lift` and `shoulder_pan`. Reconnect the full chain afterwards.

### 3. Calibrate the joints (LeRobot)

This records each joint's centre and range, so angles mean the same thing on the real arm and in the MuJoCo model:

```powershell
lerobot-calibrate --robot.type=so101_follower --robot.port=COM4 --robot.id=follower
```

1. If a calibration file already exists, type `c` and press Enter to redo it (plain Enter keeps the old one).
2. The motors go limp. Move the arm by hand to the **middle of every joint's range**: base facing straight ahead, shoulder and elbow each halfway between their end stops, wrist halfway, gripper half open. Press Enter.
3. Move **every joint except wrist roll** slowly from one end stop to the other and back (shoulder pan, shoulder lift, elbow, wrist flex, gripper fully open and fully closed). Press Enter when done. Wrist roll is a full-turn joint and is not recorded.

Always use `--robot.id=follower`: `sim2hw/hw_bridge.py` loads
`%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\follower.json`.
Without `--robot.id` the file is saved as `None.json` instead. The bridge then cannot find it and runs with default offsets, so every move lands in the wrong place.

Redo this step after replacing a servo, re-mounting a horn, or if the real arm and the simulation disagree (step 4).

### 4. Check the real arm against the simulation

```powershell
.\run.ps1 -Port COM4 -Detector none -Viewer
```

Open `http://127.0.0.1:7842/` (the coordinate-control page), click **Connect** and wait for the hardware badge to show online. Click **Home**. The real arm and the MuJoCo viewer should take the same pose. A joint that turns the wrong way or stops noticeably short means step 3 was off, so redo it.

### 5. Record the gripper positions

In the coordinate-control page, under **Gripper Positions**:

1. Click **Unlock Gripper** (only the gripper goes limp).
2. Close the jaws fully by hand with **nothing between them**, then click **Record Close**. The bridge uses this angle to tell an empty grab from a real one.
3. Open the jaws a little wider than your largest object (about 7 cm for a 6 cm cube), then click **Record Open**.
4. Click **Lock Gripper**.

### 6. Record a home pose (optional)

Move the arm to a safe resting pose with the Target jog buttons and **Run Coordinate Action**, then click **Record Current As Home**. KILL/HOME in the detector return here.

### 7. Calibrate the camera view of the board

Start the detector (`.\run.ps1 -Port COM4`). If the image is black or from the wrong camera, run `lerobot-find-cameras opencv`: it lists the cameras and saves a test picture from each to `outputs\captured_images`. Pass the board camera's index with `-Camera <index>`. On the first run it asks for two calibrations:

- **Board corners:** click the four inner corners of the taped board. The **first click is board (0, 0)**, and click 1 → 2 runs along the 21.3 cm side. Remember which physical corner you clicked first, because step 8 uses it.
- **Object height plane:** place the 6 × 6 cm reference square (top face 4 cm high) in each corner as prompted, press Enter, and click its four top corners.

Details: [CV-seg/Robotic_Arm-main/README.md](CV-seg/Robotic_Arm-main/README.md#first-run-calibration).

### 8. Record the board-to-robot points

These tell the detector where board coordinates are in the arm's frame. In the coordinate-control page:

1. Choose **Downwards**. Drive the gripper with the Target X/Y/Z buttons and **Run Coordinate Action** until the point between the jaw tips sits just above a board corner. Record in the same pose the arm uses for picking (gripper pointing down), not with the wrist tilted.
2. Under **CV Board Calibration Points**, enter the name and board coordinates and click **Record Current Arm Point**. Do all four corners (at least three):

| Name | Board X cm | Board Y cm | Corner |
|---|---|---|---|
| `Corner_1` | 0 | 0 | the corner you clicked first in step 7 |
| `Corner_2` | 21.3 | 0 | second click |
| `Corner_3` | 21.3 | 28.3 | third click |
| `Corner_4` | 0 | 28.3 | fourth click |

The points are saved in `sim2hw/cv_robot_points.json`. More detail: [sim2hw/CV_IK_MAPPING.md](sim2hw/CV_IK_MAPPING.md).

### 9. Teach a few pick spots

The corner points only give a starting map, and the first picks may miss by a centimetre or two. In the detector, pick an object (`a`). Nudge the pick until the gripper closes centred on it, then press `g`. Repeat at the yellow **teach here** marker until the panel's expected miss is small. See [Teaching the arm](CV-seg/Robotic_Arm-main/README.md#teaching-the-arm-learning-mode).

### When to redo what

| After … | Redo |
|---|---|
| Replacing a servo or re-mounting a horn | 2 (new servo only), 3, 4, then 8 and 9 |
| Moving the camera | 7 (the detector warns "board/camera moved") |
| Moving the board or the arm base | 7 and 8, then teach new spots (old ones are ignored automatically; Shift+G deletes them) |
| Changing the gripper or its jaws | 5 |
| A new session with nothing moved | Nothing; the detector checks for a moved board on startup |

### Leader arm (teleoperation only)

Only needed for [Leader-Follower Teleop Control](#leader-follower-teleop-control). Set up and calibrate the leader the same way, with its own port and id:

```powershell
lerobot-setup-motors --teleop.type=so101_leader --teleop.port=COM5      # brand-new arm only
lerobot-calibrate --teleop.type=so101_leader --teleop.port=COM5 --teleop.id=leader
```

## Browser Control

Use this when you want to control the arm from the web dashboard. This can run in simulation only, or mirror commands to real hardware if the hardware bridge is also running.

Shortcut that starts everything below and opens the dashboard: `.\run.ps1 -Port COM4 -Detector none -Dashboard` (leave out `-Port COM4` for simulation only). Full reference: [sim2hw/README.md](sim2hw/README.md).

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
