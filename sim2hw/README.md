# sim2hw — IK server and hardware bridge

This folder turns Cartesian targets (x, y, z in metres) into safe, smooth
motion of the SO-ARM101, in MuJoCo and on the real arm.

| Program | Port | Job |
|---|---|---|
| `ik_server.py` | 7842 | MuJoCo model, inverse kinematics, pick/place planning, motion pipelines. Serves the coordinate-control page. |
| `hw_bridge.py` | 7843 | Talks to the real arm's Feetech STS3215 servos through LeRobot. Streams the motion and runs the gripper. |

The camera-based object detector that sends pick/place targets here lives in
[`CV-seg/Robotic_Arm-main/`](../CV-seg/Robotic_Arm-main/README.md).

## Quick start

From the repository root, once:

```powershell
.\setup.ps1
```

Then every time:

```powershell
.\run.ps1 -Port COM4        # IK server + bridge + detector, each in its own window
.\run.ps1                   # simulation only (no real arm)
```

Or start the two servers by hand, from this folder, in separate terminals:

```powershell
..\.venv\Scripts\python.exe ik_server.py            # add --viewer for the 3-D window
..\.venv\Scripts\python.exe hw_bridge.py --port COM4
```

Start the IK server first. It mirrors every motion to the bridge when the
bridge is running; without the bridge it only moves the simulation. Find the
arm's port with `lerobot-find-port`.

Before the first hardware run, calibrate the arm with LeRobot. The bridge loads
the calibration id `follower`:

```powershell
lerobot-calibrate --robot.type=so101_follower --robot.port=COM4 --robot.id=follower
```

For a new arm, follow the full step-by-step
[first-time calibration](../README.md#first-time-calibration-so-arm101): motor
IDs, joint calibration, gripper positions, camera and board points.

## IK server (`ik_server.py`)

### Options

| Option | Default | Meaning |
|---|---|---|
| `--viewer` | off | Open the interactive MuJoCo 3-D viewer |
| `--port` | 7842 | HTTP port |
| `--scene` | `scene_full.xml` | MuJoCo scene to load |

Open `http://127.0.0.1:7842/` for the coordinate-control page (see
[COORDINATE_CONTROL.md](COORDINATE_CONTROL.md)).

### How a pick/place target is planned

`POST /coordinate_action` (and the dry-run `/coordinate_preview`) plan the whole
move before anything moves, and run exactly the joint angles they checked:

1. **Grasp first.** The target pose is solved with position as the priority;
   "gripper pointing down" uses only the arm's remaining freedom, so position
   error stays below 1 mm and any tilt is reported instead of hidden.
2. **Approach straight above it.** From the grasp, the planner climbs in 1 cm
   steps on the same arm configuration (no elbow flip near the table), up to
   `max(z + 15 cm, 18 cm)` or as high as the arm can stay within 25 mm / 25° of
   tilt. At least 4 cm of clearance is required.
3. **Jaws aligned with robot X.** For `orientation: downwards`, wrist roll is
   computed per target so the line the jaws close along is parallel to the
   robot's +X axis (`jaw_yaw_deg` changes the angle). The SO-101 gripper has
   one fixed jaw carrying the motor; `fixed_jaw_side` keeps that jaw on the same
   side (default: toward the robot base) so nearby targets never flip the wrist
   180°. The preflight reports if it had to flip.
4. **Smooth descent.** If moving straight in joint space would bow the tool
   more than 12 mm off vertical, waypoints are added. Descent and retreat each
   run as one continuous path through them (no stops).
5. **Safety checks.** Targets closer than 8 cm to the base axis are rejected.
   If the model predicts the gripper touching the arm's own shoulder (common
   below about 12 cm), the preflight warns but does not block: the real arm
   reaches further in than the model.
6. **Grip check.** After a pick's close, the server asks the bridge whether the
   jaws met the object. If they closed fully on nothing, the pipeline stops
   before lifting (`verify_grip`).

A preview's plan is cached for 120 s, so the action that follows reuses it.

### `/coordinate_action` and `/coordinate_preview` fields

| Field | Default | Meaning |
|---|---|---|
| `x`, `y`, `z` | required | Target in metres, robot frame (base axis at x = y = 0) |
| `orientation` | `free` | `downwards`, `horizontal` or `free` |
| `pre_gripper`, `post_gripper` | `hold` | `open`, `close`, `hold` or 0..1, before / after reaching the target |
| `retreat` | true | Return to the approach height afterwards |
| `approach_z` | auto | Fixed approach height (must be reachable) |
| `duration` | 7.0 | Seconds per arm motion (the detector sends 4) |
| `settle_time` | 0.4 | Pause after each arm move |
| `gripper_settle` | 0.8 | Time given to the gripper |
| `max_err_mm` | 15 | Grasp position tolerance |
| `approach_max_err_mm` | 25 | Approach/descent position tolerance |
| `max_axis_err_deg` | 10 | Grasp tilt tolerance |
| `approach_max_axis_err_deg` | 25 | Approach tilt tolerance |
| `min_approach_clearance` | 0.04 | Required height of the approach above the target (m) |
| `wrist_angle` | auto | Fix wrist roll (rad) instead of aligning the jaws |
| `jaw_yaw_deg` | 0 | Angle of the jaw closing line from robot +X |
| `fixed_jaw_side` | `base` | Motor jaw `base`-side, `away` from the base, or `auto` |
| `min_radius_m` | 0.08 | Closest allowed distance to the base axis |
| `verify_grip` | true | Stop the pick if the bridge reports an empty grip |

The response lists the planned `steps` (each move carries its `joints`) and a
`preflight` with errors (mm), tilt (deg) and any warnings.

### Endpoints

| Method | Path | Body / result |
|---|---|---|
| GET | `/` | Coordinate-control page |
| GET | `/state` | Joints, end-effector position, pipeline state and log |
| GET | `/render` | Base64 PNG of the simulation |
| GET | `/home_pose`, `/gripper_positions`, `/cv_points` | Saved poses / board calibration points |
| POST | `/coordinate_action` | Plan and run a target (fields above) |
| POST | `/coordinate_preview` | Plan only |
| POST | `/ik` | `{x, y, z, wrist_angle?, orientation?}` move the simulation (not mirrored) |
| POST | `/pipeline` | `{steps: [...]}` run move_ik / gripper / home / wait steps |
| POST | `/move` | `{joints: [...]}` set joints directly |
| POST | `/gripper` | `{value: 0..1}` (0 = open, 1 = closed) |
| POST | `/home`, `/stop` | Go home / kill switch (also stops the bridge) |
| POST | `/radius_limits` | `{z, orientation?, max_err_mm?, max_axis_err_deg?}` reachable radius band |
| POST | `/cv_points` | `{name, board_xy_cm, joints?}` record a board calibration point |
| POST | `/home_pose`, `/gripper_positions` | Save the home pose / open and close positions |
| POST | `/setcam`, `/open_viewer` | Camera angle / open the 3-D viewer |

### Files it reads and writes (this folder)

| File | Content |
|---|---|
| `scene_full.xml`, `so101_new_calib.xml`, `assets/` | MuJoCo model of the arm |
| `home_pose.json` | Saved home pose |
| `gripper_positions.json` | Saved gripper open / closed angles. The bridge uses "close" to recognise an empty grip, so save it with the jaws fully closed on nothing. |
| `cv_robot_points.json` | Board-to-robot calibration points (see [CV_IK_MAPPING.md](CV_IK_MAPPING.md)) |
| `ik_pose_points.json` | Recently solved poses, reused as IK starting points |

## Hardware bridge (`hw_bridge.py`)

The bridge connects with LeRobot's Feetech driver using the calibration in
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/follower.json`.

### Motion

- **Servo tuning on connect.** The arm joints get LeRobot's SO-101 gains
  (P 16, I 0, D 32); the factory P of 32 makes the arm shaky. They are
  re-applied on every connect without switching torque off.
- **Smooth streaming.** Trajectories are streamed at 50 Hz as plain position
  goals (servo `Goal_Time` and `Goal_Velocity` set to 0), so the servos follow
  one smooth curve instead of restarting a short move on every update. Single
  moves use a quintic profile; `/move_path` passes through waypoints on a
  shape-preserving curve that never overshoots them.

### Gripper

- **Close until contact.** A close command ramps the jaws while watching the
  gripper's load and how far it lags its command. On contact it holds a few
  degrees past the contact point (`--grip-squeeze-deg`) instead of stalling
  against the object. Closing all the way, to within about 6° of the saved
  closed angle, is reported as **empty**.
- **Protection.** LeRobot's gripper limits: 50 % torque, about 1.6 A protection
  current, 25 % overload torque. An unlimited stalled gripper sags the shared
  supply and makes the whole arm jitter while it holds something.

### Options

| Option | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port (Windows: `COM4` etc.) |
| `--baud` | 1000000 | Bus speed |
| `--hwport` | 7843 | HTTP port |
| `--p-gain` | 16 | Arm position gain. Lower if it vibrates, higher if it sags or lags |
| `--d-gain` | 32 | Arm damping. Raise if it wobbles at the end of a move |
| `--servo-accel` | 254 | Servo acceleration limit 1..254 |
| `--gripper-torque-limit` | 500 | Gripper torque limit in 0.1 % (500 = 50 %). Raise if objects slip |
| `--grip-squeeze-deg` | 5 | Hold this far past contact. More = firmer grip |
| `--grip-load-threshold` | 30 | Gripper load % that counts as contact. Lower for soft objects |
| `--no-grip-detect` | off | Close straight to the set position (old behaviour) |
| `--log-health` | 0 | Print every servo's load, current and voltage every N seconds |

### Endpoints

| Method | Path | Body / result |
|---|---|---|
| POST | `/move` | `{joints: {name: rad}}` one smooth move (duration from `/set_motion_time`) |
| POST | `/move_path` | `{waypoints: [{name: rad}], durations: [s]}` one continuous motion |
| POST | `/gripper` | `{value: 0..1, mode?}`. Closing stops on contact unless `mode: "position"` |
| POST | `/home`, `/stop` | Home pose / hold the current pose (a held object stays gripped) |
| POST | `/set_motion_time` | `{time_ms}` duration of the next `/move` |
| POST | `/gripper_torque` | `{enabled}` gripper torque on/off (move it by hand) |
| POST | `/home_pose`, `/gripper_positions` | Save poses; `/gripper_positions` without a value records the current angle |
| GET | `/ping`, `/joints` | Connection status / current joint angles (rad) |
| GET | `/health` | Per servo: load %, current A, voltage V, temperature °C |
| GET | `/grip_status` | `closing`, `holding` or `empty`, with contact angle and load |

## Troubleshooting

| Symptom | Try |
|---|---|
| Arm vibrates while holding still | `--p-gain 12` |
| Arm sags or lags under load | `--p-gain 20` |
| Arm jitters only while holding an object | Run with `--log-health 1`. If the gripper draws about 1 A and the voltage dips, lower `--gripper-torque-limit` |
| Objects slip | `--grip-squeeze-deg 8` or `--gripper-torque-limit 650` |
| "Nothing was gripped" on a real grab | Re-save the gripper "close" position with the jaws fully closed on nothing; very thin objects (< 5 mm) look empty |
| "Target is too close to the base" | The target is within 8 cm of the base axis; move the object further out |
| Preflight "model predicts shoulder/gripper contact" | Usually fine on the real arm. Keep a hand near KILL/STOP the first time |
| Bridge cannot open the port | Close other programs using it (LeRobot scripts, another bridge) and check `lerobot-find-port` |

More detail: [STARTUP.md](STARTUP.md), [COORDINATE_CONTROL.md](COORDINATE_CONTROL.md),
[CV_IK_MAPPING.md](CV_IK_MAPPING.md), [QUICK_REFERENCE.md](QUICK_REFERENCE.md).
