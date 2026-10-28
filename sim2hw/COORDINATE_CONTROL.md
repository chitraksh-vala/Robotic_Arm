# SO-ARM101 Coordinate Control Guide

Use `coordinate_control.html` when you want to command the arm by entering a target
coordinate and desired gripper state, instead of replaying recorded motions or using
teleoperation.

The coordinate controller sends one high-level command to the IK server. The IK server
turns it into a short safe sequence:

```text
approach above target -> move to target -> gripper action -> optional retreat
```

## Files Used

| File | Purpose |
|---|---|
| `coordinate_control.html` | Standalone browser UI for coordinate-based control |
| `ik_server.py` | MuJoCo IK server and coordinate-action backend |
| `hw_bridge.py` | Optional bridge that mirrors solved joint commands to the real arm |
| `scene_full.xml` | MuJoCo arm scene used by the IK server |
| `home_pose.json` | Saved custom Home pose, created after recording Home |
| `gripper_positions.json` | Saved gripper-only Open and Close calibration |

## 1. Start The IK Server

Open a terminal:

```powershell
cd sim2hw
python ik_server.py --viewer
```

Or run without the MuJoCo viewer window:

```powershell
cd sim2hw
python ik_server.py
```

The IK server runs at:

```text
http://127.0.0.1:7842
```

Keep this terminal open while using the coordinate controller.

## 2. Optional: Start The Hardware Bridge

Use this only when you want the real physical arm to move.

First find the serial port:

```powershell
lerobot-find-port
```

Then start the bridge:

```powershell
cd sim2hw
python hw_bridge.py --port COM4
```

Replace `COM4` with your actual arm port.

The hardware bridge runs at:

```text
http://127.0.0.1:7843
```

When the hardware bridge is running, coordinate actions sent to `ik_server.py` are
mirrored to the real arm through the existing pipeline/hardware path.

## 3. Open The Coordinate UI

Open this file in your browser:

```text
sim2hw/coordinate_control.html
```

You can double-click the file, or serve the folder:

```powershell
cd sim2hw
python -m http.server 8000
```

Then open:

```text
http://127.0.0.1:8000/coordinate_control.html
```

## 4. Connect To The Server

In the top-right server box, keep:

```text
http://127.0.0.1:7842
```

Click `Connect`.

Expected status:

```text
Server online
```

If `hw_bridge.py` is also running, the UI should show hardware as online or mock.

## 5. Enter A Coordinate Command

The target coordinate is in metres.

| Field | Meaning |
|---|---|
| `X m` | Forward/backward target position |
| `Y m` | Left/right lateral target position |
| `Z m` | Height target. The UI accepts `-0.05` to `0.65` m for low pickup tests |
| `Before move` | Gripper state before motion starts: `Open`, `Hold`, or `Close` |
| `After reach` | Gripper state after reaching the target: `Open`, `Hold`, or `Close` |
| `Gripper orientation` | IK tool direction: `Free`, `Downwards`, or `Horizontal` |
| `Approach Z m` | Height used before descending to the target |
| `Wrist roll rad` | Wrist-roll angle, from `-2.74` to `2.84` radians. `1.57` is useful for horizontal gripper fingers |
| `Retreat after action` | Move back up to `Approach Z` after the gripper action |
| `Move duration s` | Duration of each Cartesian move segment |
| `Settle s` | Pause after each move |
| `Max IK error mm` | Reject command if IK error is above this threshold |

The controller also checks radial distance from the base:

```text
radius = sqrt(x^2 + y^2)
```

The default minimum radius is `0.08 m` (`min_radius_m` overrides it per request).
For example, `x=0`, `y=0.06` is only `0.06 m` from the base, so it is rejected
before motion starts. Between about 0.08 and 0.12 m the MuJoCo model may predict
the gripper touching the arm's own shoulder; the preflight then shows a warning
instead of blocking, because the real arm reaches closer than the model. Use a
comfortable starting target such as `x=0.20`, `y=0.00`, `z=0.15`.

The UI also shows the usable radius band for the current `Z`. Low pickup targets are
allowed down to `z=-0.05`, but they still need to pass `Check Reach` before running
on hardware.

Orientation modes:

```text
Free       = solve position only; widest reachable workspace
Downwards  = point the gripper tool axis toward the base/table
Horizontal = point the gripper tool axis outward in the XY plane
```

`Downwards` and `Horizontal` are solved inside IK, so they can reduce the reachable
workspace. Use `Check Reach` after changing orientation.

Enter wrist roll in radians, not degrees. For example:

```text
37 degrees = 0.65 radians
```

Example pick/grasp command:

```text
X m:             0.25
Y m:             0.00
Z m:             0.08
Before move:     Open
After reach:     Close
Orientation:     Downwards
Approach Z m:    0.15
Wrist roll rad:  1.57
Retreat:         checked
Move duration s: 3.0
Settle s:        0.8
Max IK error mm: 15
```

Then click:

```text
Run Coordinate Action
```

## 6. What Happens Internally

For the example above, the server generates a pipeline similar to:

```json
[
  {
    "type": "gripper",
    "label": "Open gripper before move",
    "value": 0.0
  },
  {
    "type": "move_ik",
    "label": "Approach target",
    "x": 0.25,
    "y": 0.0,
    "z": 0.15,
    "orientation": "downwards",
    "wrist_angle": 1.57
  },
  {
    "type": "move_ik",
    "label": "Move to coordinate",
    "x": 0.25,
    "y": 0.0,
    "z": 0.08,
    "orientation": "downwards",
    "wrist_angle": 1.57
  },
  {
    "type": "gripper",
    "label": "Close gripper after reaching target",
    "value": 1.0
  },
  {
    "type": "move_ik",
    "label": "Retreat above target",
    "x": 0.25,
    "y": 0.0,
    "z": 0.15,
    "orientation": "downwards",
    "wrist_angle": 1.57
  }
]
```

The UI shows the generated steps and the preflight IK error before motion begins.

## Kill Switch

Click `Kill Switch` to stop the current coordinate pipeline.

The simulator immediately cancels interpolation and holds the current joint positions.
If `hw_bridge.py` is running, the hardware bridge reads the current motor positions and
sets those positions as the new servo goals.

This is a software stop. Keep a physical power cutoff available while testing the real arm.

## Recording A Custom Home Pose

You can make the current arm position the new Home position.

1. Move the arm to the desired resting pose.
2. Close or open the gripper as desired for Home.
3. In `coordinate_control.html`, click `Record Current As Home`.
4. The saved joint values appear in the `Home Pose` panel.
5. Press `Home` to test the saved pose.

The pose is saved to:

```text
sim2hw/home_pose.json
```

The saved pose is reused after restarting `ik_server.py`. The hardware bridge also reads
the same file when `/home` is called.

If the hardware bridge is online, `Record Current As Home` records the real arm's current
joint readings from `hw_bridge.py`. If the hardware bridge is offline, it records the
simulator's current joint pose instead.

Gripper convention:

```text
0.0 = partial open
1.0 = fully closed
```

Internally, both the simulator and hardware bridge map `open` and `close` through
saved gripper positions. Use the Gripper Positions panel to tune these values on the
real arm.

## Recording Gripper Open And Close

Use this when `Open` is too wide or `Close` does not clamp enough.

1. Start `hw_bridge.py` so the UI can read the real gripper motor.
2. In `coordinate_control.html`, click `Unlock Gripper`.
3. Move only the gripper by hand to the desired open position.
4. Click `Record Open`.
5. Move only the gripper by hand to the desired closed position.
6. Click `Record Close`.
7. Click `Lock Gripper`.
8. Test `Open` and `Close` from the coordinate action panel.

The saved gripper positions are written to:

```text
sim2hw/gripper_positions.json
```

They are reused after restarting both `ik_server.py` and `hw_bridge.py`.

`Unlock Gripper` disables torque only on the gripper motor when the installed hardware
driver supports it. The rest of the arm remains held.

You can inspect the saved Home pose directly:

```text
http://127.0.0.1:7842/home_pose
```

You can inspect saved gripper positions directly:

```text
http://127.0.0.1:7842/gripper_positions
```

## Direct API Usage

You can call the backend without the browser UI:

```powershell
$body = @{
  x = 0.25
  y = 0.00
  z = 0.08
  gripper = "close"
  orientation = "downwards"
  approach_z = 0.15
  wrist_angle = 1.57
  duration = 1.5
  settle_time = 0.6
  max_err_mm = 15
  retreat = $true
} | ConvertTo-Json

Invoke-RestMethod `
  -Uri "http://127.0.0.1:7842/coordinate_action" `
  -Method POST `
  -ContentType "application/json" `
  -Body $body
```

Valid gripper values:

```text
open   = partial open
close
hold
```

For coordinate actions, use:

```json
{
  "pre_gripper": "open",
  "post_gripper": "close"
}
```

The older `gripper` field is still accepted as the after-reaching action for compatibility.

You can also send numeric gripper values with `gripper_value`:

```json
{
  "x": 0.25,
  "y": 0.0,
  "z": 0.08,
  "gripper_value": 0.5
}
```

Here:

```text
0.0 = partial open
1.0 = fully closed
```

## Recommended Safety Flow

For the first test:

1. Run only the IK server, without hardware.
2. Open `coordinate_control.html`.
3. Try a high target first, such as `x=0.20`, `y=0.00`, `z=0.25`.
4. Watch the MuJoCo viewer or rendered preview.
5. Lower `z` gradually.
6. Start `hw_bridge.py` only after the simulated motion looks correct.

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| `Server offline` | `ik_server.py` is not running | Start `python ik_server.py --viewer` from `sim2hw` |
| UI connects but no hardware moves | Hardware bridge is not running | Start `python hw_bridge.py --port COM4` |
| Command rejected with IK error | Target may be unreachable | Use the usable radius band, increase Z, reduce reach, or raise `Max IK error mm` only for testing |
| Arm moves too fast | Move duration is low | Increase `Move duration s` |
| Gripper orientation is wrong | Wrist roll angle needs adjustment | Try `1.57`, `0`, or `-1.57` |
| Target is near table | Approach path may still collide | Increase `Approach Z m` before using hardware |

## Notes

- Coordinates are world coordinates in metres, matching the MuJoCo scene.
- The coordinate action does not do camera detection or object recognition.
- The server validates basic XYZ bounds and IK error, but it does not yet perform full
  collision planning.
- Keep an emergency stop or power cutoff available when testing on the physical arm.
