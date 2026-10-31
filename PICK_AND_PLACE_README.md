# Vision Pick-and-Place: Operator Guide

This guide takes you from an unpacked SO-ARM101 to picking and placing objects
with the camera. It covers installing the software, calibrating the arm,
starting the servers, calibrating the camera and board, and every function of
the detector window: pick and place, place spots by colour, the stacker, and
teaching the arm to be accurate.

Read sections 1–3 once. The first time you set up an arm, follow sections 4–8
in order. For daily use, go to the [daily checklist](#16-daily-checklist).

## Contents

1. [How the system works](#1-how-the-system-works)
2. [What you need](#2-what-you-need)
3. [Install the software (once)](#3-install-the-software-once)
4. [Calibrate the arm's joints (LeRobot)](#4-calibrate-the-arms-joints-lerobot)
5. [Start the system](#5-start-the-system)
6. [Check the arm, record the gripper and home pose](#6-check-the-arm-record-the-gripper-and-home-pose)
7. [Calibrate the camera view of the board](#7-calibrate-the-camera-view-of-the-board)
8. [Record the board-to-robot points](#8-record-the-board-to-robot-points)
9. [The detector window](#9-the-detector-window)
10. [Pick and place](#10-pick-and-place)
11. [Place spots by colour](#11-place-spots-by-colour)
12. [Stacker mode](#12-stacker-mode)
13. [Teaching the arm (learning mode)](#13-teaching-the-arm-learning-mode)
14. [Options reference](#14-options-reference)
15. [Files reference](#15-files-reference)
16. [Daily checklist](#16-daily-checklist)
17. [When to redo what](#17-when-to-redo-what)
18. [Troubleshooting](#18-troubleshooting)
19. [Safety](#19-safety)

---

## 1. How the system works

Three programs run at once, each in its own window:

```text
 camera ──► object detector ──HTTP──► IK server ──HTTP──► hardware bridge ──USB──► SO-ARM101
            (finds objects,           (MuJoCo model:      (drives the
             you pick here)            plans every move)   servos)
```

| Program | File | Port | Job |
|---|---|---|---|
| Object detector | `CV-seg/Robotic_Arm-main/object_detector_standalone.py` | — | Shows the camera, finds and measures objects, converts their board position to a robot target, and is where you operate everything |
| IK server | `sim2hw/ik_server.py` | 7842 | Holds a MuJoCo model of the arm. Plans each pick or place before anything moves, rejects targets the arm cannot reach, and serves the coordinate-control web page |
| Hardware bridge | `sim2hw/hw_bridge.py` | 7843 | Connects to the real arm over USB with LeRobot's Feetech driver and streams the planned motion to the servos. Without it, only the simulation moves |

**Coordinates.** Two frames are used:

- **Board (cm):** the taped board is 21.3 × 28.3 cm. Board (0, 0) is the corner
  you click first when calibrating the camera. Board X runs along the 21.3 cm
  side, board Y along the 28.3 cm side.
- **Robot (m):** origin on the arm's base axis, X forward, Y to the arm's
  left, Z up. The nudge keys move targets in this frame.

**Where accuracy comes from.** Four calibrations stack on top of each other:

1. **Joint calibration** (LeRobot): servo readings mean the same angles as in
   the MuJoCo model.
2. **Camera calibration:** image pixels map to board centimetres.
3. **Board-to-robot points:** board centimetres map to robot coordinates.
4. **Taught spots:** your corrections teach the remaining few millimetres.

A mistake in an early step cannot be fully fixed by a later one, so do them in
order.

---

## 2. What you need

**Hardware**

- SO-ARM101 follower arm with its 12 V supply and USB cable.
- USB camera mounted above the board, looking down at the whole board. Keep it
  fixed: moving it means recalibrating the camera.
- The board: white, 21.3 × 28.3 cm inside a red tape border, fixed in place
  in front of the arm.
- A 6 × 6 cm reference square whose top face is 4 cm above the board (for the
  camera's height calibration).
- Objects to pick: 1–15 cm across and **4 cm tall** (the detector assumes one
  height, `--object-height`), small enough to fit the open gripper. Coloured or
  dark objects work best; white objects are off by default (they look like the
  board).

**Computer**

- Windows 10/11 with PowerShell.
- Python 3.12 or 3.13 from python.org (tick *Add to PATH*). 3.10–3.11 only
  works for simulation (`-SimOnly`).
- Several GB of free disk and a good connection: LeRobot installs PyTorch, a
  large download.

---

## 3. Install the software (once)

Open PowerShell in the repository folder and run:

```powershell
.\setup.ps1
```

It creates a virtual environment in `.venv`, installs `requirements.txt`
(MuJoCo, NumPy, OpenCV, Pillow) and `requirements-hardware.txt` (LeRobot with
the Feetech driver). It then replaces LeRobot's window-less OpenCV with the
normal build, so the detector window can open, and checks that the MuJoCo
scene loads.

| Option | Use |
|---|---|
| `-SimOnly` | No real arm: skips LeRobot and PyTorch |
| `-Python "C:\...\python.exe"` | Use a specific Python |
| `-Recreate` | Delete and rebuild `.venv` |

If PowerShell refuses to run scripts:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1
```

The LeRobot commands used below (`lerobot-find-port`, `lerobot-calibrate`, …)
are installed in `.venv\Scripts`. Activate the environment once per terminal so
they can be run by name:

```powershell
.\.venv\Scripts\Activate.ps1
```

---

## 4. Calibrate the arm's joints (LeRobot)

Every program depends on this. If it is wrong, the arm goes to the wrong place
however well everything else is calibrated. Power the arm and plug in its USB
cable first.

### 4.1 Find the arm's serial port

```powershell
lerobot-find-port
```

Unplug the arm's USB cable when asked and press Enter. The port that
disappears (for example `COM6`) is the arm. Use it everywhere below in place of
`COM6`.

### 4.2 Set the motor IDs (brand-new arm only)

Skip this if the arm has been used before. New STS3215 servos all share one
ID, so each must be given its own:

```powershell
lerobot-setup-motors --robot.type=so101_follower --robot.port=COM6
```

When asked, connect **only that one motor** to the controller board and press
Enter. It goes gripper, wrist roll, wrist flex, elbow flex, shoulder lift,
shoulder pan. Reconnect the full chain afterwards.

### 4.3 Calibrate

```powershell
lerobot-calibrate --robot.type=so101_follower --robot.port=COM6 --robot.id=follower
```

1. If a calibration for `follower` already exists, LeRobot asks whether to use
   it. **Plain Enter keeps the old one. Type `c` and press Enter to calibrate
   again.**
2. The motors go limp. Move the arm by hand to the **middle of every joint's
   range** and press Enter:
   - base facing straight ahead,
   - upper arm about halfway up (not folded down on its rest),
   - elbow bent about 90°,
   - wrist straight, in line with the forearm,
   - gripper half open.
3. Move **every joint except wrist roll** slowly from one end stop to the
   other: shoulder pan, shoulder lift, elbow, wrist flex, and the gripper fully
   open and fully closed. Press Enter. Wrist roll turns all the way round and is
   not recorded.

**Always use `--robot.id=follower`.** The bridge loads
`%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\follower.json`.
Without the id the file is saved as `None.json`, the bridge does not find it,
and every move lands in the wrong place.

### 4.4 Check the calibration file

A bad middle pose can make a joint's recorded range wrap past the servo's
0/4095 point. The arm then misses by tens of degrees. One symptom: asked to go
down to the board (z = 0), it grabs high above the object. Check the ranges
after every calibration:

```powershell
python -c "import json,pathlib; d=json.load(open(pathlib.Path.home()/'.cache/huggingface/lerobot/calibration/robots/so_follower/follower.json')); [print(k, v['range_min'], v['range_max'], round((v['range_max']-v['range_min'])*360/4095),'deg') for k,v in d.items()]"
```

Only `wrist_roll` may show about 360°. Every other joint should be well below
that (shoulder lift about 200–220°). A joint at about 359° wrapped: calibrate
again (type `c`) with that joint closer to the middle of its travel.

### 4.5 Restart the bridge

The bridge reads the calibration only when it connects. Restart it after every
calibration (close its window and start it again, or rerun `run.ps1`).

---

## 5. Start the system

### 5.1 The easy way: `run.ps1`

From the repository folder:

```powershell
.\run.ps1 -Port COM6           # IK server + bridge + detector in pick/place mode
.\run.ps1                      # no real arm: IK server + detector in preview mode
```

It starts each program in its own window, in order, and waits for each server
to answer before starting the next. Close a window (or press Ctrl+C in it) to
stop that part.

| Option | Default | Use |
|---|---|---|
| `-Port COM6` | none | The arm's port. Without it nothing moves the real arm |
| `-Detector` | `pick-place` with `-Port`, else `preview` | `pick-place`, `preview` (reach checks only), `detect` (camera only) or `none` |
| `-Camera 1` | 1 | Camera index for the detector |
| `-Viewer` | off | Open the MuJoCo 3-D viewer |
| `-Dashboard` | off | Also serve and open the browser dashboard |
| `-IkArgs "..."` | | Extra options for `ik_server.py` |
| `-BridgeArgs "..."` | | Extra options for `hw_bridge.py`, e.g. `"--log-health 1"` |
| `-DetectorArgs "..."` | | Extra options for the detector, e.g. `"--exposure -7"` |
| `-DryRun` | off | Print the commands without starting anything |

`Get-Help .\run.ps1 -Detailed` lists them too.

### 5.2 By hand (three terminals)

Start them in this order. Set UTF-8 output in each terminal first, or the IK
server can crash when it prints status symbols:

```powershell
$env:PYTHONIOENCODING = "utf-8"
```

**Terminal 1, IK server** (from `sim2hw`):

```powershell
cd sim2hw
..\.venv\Scripts\python.exe ik_server.py            # add --viewer for the 3-D window
```

Wait until it prints `HTTP port : 7842`.

**Terminal 2, hardware bridge** (from `sim2hw`):

```powershell
cd sim2hw
..\.venv\Scripts\python.exe hw_bridge.py --port COM6
```

It should print `Connected via lerobot to COM6`.

**Terminal 3, detector** (from the repository folder):

```powershell
.\.venv\Scripts\python.exe CV-seg\Robotic_Arm-main\object_detector_standalone.py --ik-pick-place --camera 1
```

The IK server copies every motion to the bridge when the bridge is running.
Without the bridge, only the simulation moves.

### 5.3 Check everything is up

| Check | Expected |
|---|---|
| `http://127.0.0.1:7842/` in a browser | Coordinate-control page loads |
| `http://127.0.0.1:7842/state` | JSON with joints and the pipeline log |
| `http://127.0.0.1:7843/ping` | Bridge is connected |
| `http://127.0.0.1:7843/health` | Per servo: load, current, voltage, temperature |
| Detector window | Camera image with the board outlined in cyan and a side panel |

---

## 6. Check the arm, record the gripper and home pose

Do this after a new joint calibration. Start the IK server and bridge
(`.\run.ps1 -Port COM6 -Detector none -Viewer`), open
`http://127.0.0.1:7842/`, keep the server address `http://127.0.0.1:7842` and
click **Connect**. Wait for the hardware badge to show online.

### 6.1 Real arm against the simulation

Click **Home**. The real arm and the MuJoCo viewer should take the same pose.
A joint that turns the wrong way or stops noticeably short means the joint
calibration is off: repeat [section 4](#4-calibrate-the-arms-joints-lerobot).

### 6.2 Gripper positions

Under **Gripper Positions**:

1. Click **Unlock Gripper** (only the gripper goes limp).
2. Close the jaws fully by hand with **nothing between them** and click
   **Record Close**. The bridge uses this angle to tell an empty grab from a
   real one.
3. Open the jaws a little wider than your largest object (about 7 cm for a
   6 cm cube) and click **Record Open**.
4. Click **Lock Gripper**.

Saved in `sim2hw/gripper_positions.json`.

### 6.3 Home pose (optional)

Move the arm to a safe resting pose (Target jog buttons and **Run Coordinate
Action**), then click **Record Current As Home**. KILL/HOME in the detector
return here. Saved in `sim2hw/home_pose.json`. Re-record it after a new joint
calibration, because the saved joint angles depend on it.

---

## 7. Calibrate the camera view of the board

Start the detector (`.\run.ps1 -Port COM6`). On the first run it asks for two
calibrations.

**Wrong or black camera?** Run `lerobot-find-cameras opencv`. It lists the
cameras and saves a test picture from each to `outputs\captured_images`. Pass
the board camera's index with `-Camera <index>` (detector: `--camera`). If the
image is black, try `-DetectorArgs "--backend msmf"` or `"--backend any"`.

### 7.1 Board corners

Click the four **inner** corners of the taped board, in order around the
board:

1. Corner 1: this becomes board **(0, 0)**.
2. Corner 2: the other end of the **21.3 cm** side.
3. Corner 3: the other end of the 28.3 cm side.
4. Corner 4: the last corner.

Enter accepts, `u` undoes a click, `r` restarts, Esc cancels. **Remember which
physical corner you clicked first**: section 8 uses it, and every later
recalibration (`c` in the detector) must start at the same corner. A reference
image is saved too, and the panel warns when the board or camera moves by more
than 8 px.

Saved in `workspace_calibration.json` (+ `workspace_calibration_reference.png`).

### 7.2 Object height plane

Objects are measured on their top face, 4 cm above the board. That face looks
bigger and shifted compared with the board surface, so the detector needs a
second calibration at that height:

1. Place the 6 × 6 cm reference square snug in the corner shown by the yellow
   outline, edges flush with the board edges.
2. Press Enter or Space when it is still.
3. Click the square's four top corners.
4. Repeat for all four corners.

Saved in `workspace_height_calibration.json`. Redo it with
`--recalibrate-height`. If the detector prints *"the height calibration
disagrees with the corners by N cm"*, redo it: positions measured on objects
will otherwise be off.

### 7.3 Exposure

If the panel warns **Overexposed**, part of the board is pure white and pale
colours are lost. Darken the camera with `-DetectorArgs "--exposure -7"`
(DirectShow: about −4 bright to −9 dark).

---

## 8. Record the board-to-robot points

These tell the detector where board positions are in the arm's frame. You
record at least three board points (the four corners are easiest) by moving
the real arm onto them.

In the coordinate-control page (`http://127.0.0.1:7842/`, connected, hardware
online):

1. Set **Gripper orientation** to **Downwards**. Record in the same pose the
   arm picks in. Points recorded with a tilted wrist cause centimetre-level
   misses later.
2. Drive the gripper with the Target X/Y/Z buttons and **Run Coordinate
   Action** until the point between the jaw tips is just above a board corner.
3. Under **CV Board Calibration Points**, enter the name and board coordinates
   and click **Record Current Arm Point**:

| Name | Board X cm | Board Y cm | Physical corner |
|---|---|---|---|
| `Corner_1` | 0 | 0 | the corner you clicked first in 7.1 |
| `Corner_2` | 21.3 | 0 | second click |
| `Corner_3` | 21.3 | 28.3 | third click |
| `Corner_4` | 0 | 28.3 | fourth click |

The points are saved in `sim2hw/cv_robot_points.json`;
`sim2hw/cv_ik_mapping.json` points the detector at that file. If the bridge is
offline, the points come from the simulation and do not calibrate the real
arm. More detail: [sim2hw/CV_IK_MAPPING.md](sim2hw/CV_IK_MAPPING.md).

This is only the starting map. Expect the first picks to miss by 1–2 cm until
you teach a few spots ([section 13](#13-teaching-the-arm-learning-mode)).

---

## 9. The detector window

The left side is the camera image:

| Mark | Meaning |
|---|---|
| Cyan outline | The calibrated board |
| Magenta outline, `1. red square` | A detected object, its number, colour and shape |
| Yellow outline | The selected object |
| Red outline | Too close to the arm's base to reach (within 8.5 cm) |
| Yellow `+` **teach here** | Where to teach the next spot (learning mode) |
| Cross, `red place` | A place spot for that colour |
| Yellow circle, `stack` | The spot of a stack being built |

The right side is the panel. Click its buttons with the mouse:

| Part | Content |
|---|---|
| **KILL / STOP [s]** | Stops the arm at once. A held object stays gripped |
| **HOME [h]** | Stops, then moves to the saved home pose |
| **DELETE OLD SPOTS** | Deletes taught spots from earlier sessions (click twice; [13.6](#136-deleting-old-learning)) |
| Warning | Moved board, overexposure, … |
| Objects | Each object: number, colour, shape, size, board position, and where it will be placed (`-> red spot`) |
| Arm / IK | The last result or error. Errors say which target failed and what to do |
| Status | Object count, FPS, scale, light level, noise filtered, IK fit |
| Nudge | Learning mode, gripper side, the learned model and its typical miss, current nudges |
| Place spots | The spot for each colour and for other colours |
| Keys | The keys available now |

**The window must have keyboard focus.** Click on it before pressing keys,
otherwise they go to another window.

An object is listed when its centre is well inside the board
(`--edge-margin`), it stays still for a few frames (`--confirm-frames`), and it
is 1–15 cm across. Cast shadows, glare streaks and the tape are filtered out;
the Status line's "Noise filtered" counts what was rejected and why. Keep
hands and the arm out of the camera view while selecting, as they can hide or
move objects.

---

## 10. Pick and place

The detector must run in pick/place mode (`.\run.ps1 -Port COM6`). In preview
mode, picks, nudges and clicks are disabled.

1. Clear the arm's path. Keep a hand near KILL (`s`).
2. Select an object with `1`–`9` (its number in the Objects list).
3. Press `p` (preview). The IK server checks that both the pick and the place
   spot are reachable with the gripper pointing down. Nothing moves.
4. Press `a`. The arm checks both targets again, then:
   - opens the gripper, approaches from above and descends to 1 cm above the
     board (`--grasp-height`),
   - closes until it feels the object; if the jaws close on nothing, it stops
     with "nothing was gripped" instead of lifting,
   - lifts, moves over the place spot, lowers, releases and lifts away.
5. Select the next object and press `a` again. Each `a` moves one object.

**Where it places:** at the spot set for the object's colour; otherwise at the
spot set for "other colours"; otherwise just inside Corner 2, the board's
(21.3, 0) corner, inset so the whole object stays on the board.

**Gripper side (`w`).** The SO-101 gripper has one fixed jaw carrying the
motor. By default that jaw faces the robot base. `w` switches it to face away.
Each side learns its corrections separately. Use one side consistently.

**Speed:** `--move-duration` (default 4 s per motion). Use 6 for first tests.

---

## 11. Place spots by colour

Each colour can have its own place spot, on the board or off it (for example
a bin beside the board).

**Set a spot for a colour**

1. Put an object of that colour on the board and select it (`1`–`9`).
2. Press `d`. The status line asks you to click.
3. Click where the object's **centre** should go, anywhere in the camera
   image. `d` again cancels.
4. The detector checks straight away whether the arm can reach the spot and
   shows either "Place spot saved and reachable" or a warning with the reason
   (too close to the base, too far round to the side). If you get a warning,
   press `d` and click a few centimetres further from the base.

**Spot for all other colours:** press `d` with **no** object selected, then
click.

**Remove a spot:** Shift+`D` (with an object selected: its colour; with none:
the other-colours spot).

**Details**

- Colour names are the detector's labels: `red`, `orange`, `yellow`, `lime`,
  `green`, `teal`, `cyan`, `blue`, `indigo`, `purple`, `magenta`, `pink`,
  `black`, `gray`, `white`. A dull `dark green` object uses a `dark green` spot
  if there is one, otherwise the `green` spot.
- Off the board, the target is extrapolated from the board map and assumes the
  surface is level with the board. Check it with `p`. If drops land off target,
  teach the drop ([13.4](#134-teaching-a-drop-spot)).
- All objects of one colour go to the same spot. Move a placed object away
  before placing the next one of that colour.
- The spots are saved in `sim2hw/cv_color_places.json` as board centimetres
  and can be edited by hand (restart the detector afterwards):

```json
{
  "default": null,
  "colors": {
    "red": [4.0, 4.0],
    "yellow": [16.5, 34.5]
  }
}
```

`"default": null` means other colours go to Corner 2.

---

## 12. Stacker mode

Stacks every visible object, one on top of another, at a point you click.

1. Make sure all the objects are listed in the panel.
2. Press `b`, then click where the stack should be (on or off the board).
   Click **on an object** to stack the others on top of it; that object stays
   where it is.
3. Before anything moves, the arm checks every pick and the top of the stack.
   An unreachable object stops it before it starts.
4. It moves the objects nearest the stack first. The panel shows progress
   ("Stacker 2/3: …") and a yellow circle marks the spot.

**Stop:** `b` stops after the object being moved; `s` / KILL stops at once.

**Heights:** each level is `--object-height` (4 cm) higher than the one below.
Objects above the first are released `--stack-clearance` (0.5 cm) above the
one below. All objects must be the same height. A tall stack can go out of
reach; the check in step 3 reports that.

Object positions are taken when you click, so do not move objects while it
runs. Objects within 8.5 cm of the base are skipped.

---

## 13. Teaching the arm (learning mode)

The board-to-robot points and the arm model are a few millimetres to 2 cm off
the real arm. Teaching saves where the gripper really has to go at a few board
spots, and the detector learns the error over the whole board from them.
Learning mode is on by default (`t` toggles it).

### 13.1 Teaching a spot (recommended way: aim)

1. Put an object at the yellow **teach here** marker and select it.
2. Press `f` (aim). The arm hovers the open gripper 1 cm above the object
   (`--aim-clearance`).
3. Nudge until the jaws are centred over the object. The arm moves after every
   nudge (about 5 s), and quick nudges are combined into one move:

   | Keys | Nudge (robot frame) | Step |
   |---|---|---|
   | `l` / `j` | +X / −X | 5 mm |
   | `i` / `k` | +Y / −Y | 5 mm |
   | `u` / `o` | +Z / −Z | 2 mm |
   | Shift + the key | same, fine | 1 mm |
   | `0` | reset the nudge | |

4. Press `g` to save the spot. If it was already centred, press `g` without
   nudging: that confirms the spot, which teaches just as much.
5. Select an object at the next marker and repeat.

Teach 4–6 spots spread over the board. The panel's Nudge section shows
"Learned arm model from N spots, typical miss ~X mm". Stop when that is small
(a few mm) and picks land centred.

**The old way** also works: `a` (pick), watch the miss, nudge, `a` again, and
`g` once it grips centred. Aiming avoids a full pick-and-place for every nudge.

### 13.2 The teach-here marker

The marker spreads taught spots over the board. It goes to the quarter of the
reachable board with the fewest spots, at its emptiest point, and keeps at
least 12 cm from the base (closer in, the gripper can hit the shoulder and
spots teach little). It is hidden once every area has a spot within 4 cm.

### 13.3 How the learning works

- **1 spot:** the whole board is shifted by that correction.
- **2 spots:** the board's shift and rotation are corrected.
- **3+ consistent spots:** a model of the real error is fitted: the board
  placed rigidly at its true size, plus the arm's pan and reach errors. Each
  taught spot then replays exactly. If simply blending the corrections
  predicts better, that is used instead.
- **Moved setup:** spots that disagree with the newest one by more than
  2.5 cm (taught before the board, camera or arm moved) are ignored and
  counted on the panel. Shift+`G` deletes them.
- **Sessions:** spots taught more than an hour before the newest one count 10%
  in the fit, so after a small change the new spots take over within a few
  teaches.
- Objects overhanging the board edge count less (their visible centre is
  biased).
- Re-teaching within 1.5 cm of a saved spot replaces it.

### 13.4 Teaching a drop spot

Press `m` until the Nudge title says **DROP**. Select an object whose colour
goes to the spot you want to correct, press `f` to aim over that place spot,
nudge, and press `g`. Or run `a`, nudge the drop, run `a` again, and press `g`.

### 13.5 Shift everything (ALL)

If every pick and drop misses the same way, press `m` until the mode reads
**ALL** and nudge. The shift applies to every target at once, is saved, and is
shown as "All targets" on the panel. `0` in ALL mode resets it. Taught spots
are stored without it, so it never counts twice.

### 13.6 Deleting old learning

- **DELETE OLD SPOTS** (panel button): click it, check the counts on the
  status line, and click again within 4 s. It deletes every spot taught before
  the current session and keeps the current session's spots. Use it after a
  joint recalibration or after moving the board. The previous file is saved as
  `sim2hw/cv_pick_corrections.json.bak`. To undo, copy the backup over the file
  and restart the detector. Note that the "current session" is whatever you
  taught most recently: teach one spot and click it, and only that spot stays.
- **Shift+`G`:** deletes only the spots the model ignores as inconsistent.
- **Start from nothing:** run with `--clear-ik-corrections`, which ignores the
  file for that run. Careful: the first `g` in such a run overwrites the file
  with only the new spot, so copy the file first if you may want the old spots
  back. Or delete `sim2hw/cv_pick_corrections.json`.

### 13.7 Learning mode off

`t` (or `--learning-mode off`): everything learned is still used, but `g`,
Shift+`G` and the marker are disabled. Use it for demonstrations so a stray key
cannot change the learning.

---

## 14. Options reference

### 14.1 Detector (`object_detector_standalone.py`)

Pass them through `run.ps1` with `-DetectorArgs "..."`.

| Option | Default | Meaning |
|---|---|---|
| `--ik-pick-place` | | Pick/place mode: picks, nudges, learning, buttons, clicks |
| `--ik-preview` | | Preview mode: reach checks only (`p`) |
| `--camera` | 1 | Camera index |
| `--backend` | dshow | Capture backend (`msmf` or `any` if the image is black) |
| `--width`, `--height` | 1280, 720 | Capture size |
| `--exposure` | auto | Manual exposure (DirectShow ≈ −4 bright … −9 dark) |
| `--lock` | off | Try to fix focus and white balance |
| `--board-length`, `--board-width` | 21.3, 28.3 | Board size in cm |
| `--object-height` | 4.0 | Object top-face height in cm |
| `--grasp-height` | 1.0 | Gripper height above the board when grasping (cm) |
| `--move-duration` | 4 | Seconds per arm motion |
| `--aim-clearance` | 1.0 | Aim (`f`): hover height above the object top (cm) |
| `--stack-clearance` | 0.5 | Stacker: release height above the object below (cm) |
| `--feedback-step-xy-mm`, `--feedback-step-z-mm`, `--feedback-fine-step-mm` | 5, 2, 1 | Nudge steps |
| `--learning-mode` | on | Start with learning on or off |
| `--gripper-motor-side` | base | `base`, `away` or `auto` |
| `--color-places` | `sim2hw/cv_color_places.json` | Place spots per colour |
| `--ik-corrections` | `sim2hw/cv_pick_corrections.json` | Taught spots |
| `--clear-ik-corrections` | off | Ignore taught spots for this run |
| `--ik-mapping` | `sim2hw/cv_ik_mapping.json` | Board-to-robot map |
| `--ik-url` | http://127.0.0.1:7842 | IK server |
| `--recalibrate`, `--recalibrate-height` | off | Redo the corner / height calibration |
| `--min-object-cm`, `--max-object-cm` | 1, 15 | Accepted object size |
| `--confirm-frames` | 4 | Frames an object must persist before it is listed |
| `--edge-margin` | 30 | Ignore objects whose centre is this close (px) to the board edge |
| `--roi-inset` | 10 | Keep red/orange/pink detections this far (px) from the red tape |
| `--white-objects` | off | Also detect white objects (glare looks the same) |

### 14.2 IK server (`ik_server.py`)

| Option | Default | Meaning |
|---|---|---|
| `--viewer` | off | Open the MuJoCo 3-D viewer |
| `--port` | 7842 | HTTP port |
| `--scene` | `scene_full.xml` | MuJoCo scene |

Planning details and the HTTP API: [sim2hw/README.md](sim2hw/README.md).

### 14.3 Hardware bridge (`hw_bridge.py`)

| Option | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port (`COM6` on Windows) |
| `--p-gain` | 16 | Lower if the arm vibrates, higher if it sags |
| `--d-gain` | 32 | Raise if it wobbles at the end of a move |
| `--servo-accel` | 254 | Servo acceleration limit 1..254 |
| `--gripper-torque-limit` | 500 | Gripper torque in 0.1 % (500 = 50 %). Raise if objects slip |
| `--grip-squeeze-deg` | 5 | Hold this far past contact. More = firmer |
| `--grip-load-threshold` | 30 | Load % that counts as contact. Lower for soft objects |
| `--no-grip-detect` | off | Close straight to the set position |
| `--log-health` | 0 | Print every servo's load, current and voltage every N s |

---

## 15. Files reference

| File | Made by | Content | Redo when |
|---|---|---|---|
| `%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\follower.json` | `lerobot-calibrate` (4) | Joint offsets and ranges | Servo replaced, horn remounted, arm misses badly |
| `sim2hw/gripper_positions.json` | coordinate page (6.2) | Gripper open / closed angles | Gripper or jaws changed |
| `sim2hw/home_pose.json` | coordinate page (6.3) | Home pose | After a joint calibration |
| `workspace_calibration.json` + `_reference.png` | detector (7.1) | Board corners, reference image | Camera or board moved |
| `workspace_height_calibration.json` | detector (7.2) | 4 cm plane | Camera moved, or the "disagrees" warning |
| `sim2hw/cv_robot_points.json` | coordinate page (8) | Board-to-robot points | Board or arm base moved |
| `sim2hw/cv_ik_mapping.json` | by hand | Points file name and tool offset | Rarely |
| `sim2hw/cv_pick_corrections.json` (+ `.bak`) | detector `g` (13) | Taught spots and the ALL shift | Delete old spots after big changes |
| `sim2hw/cv_color_places.json` | detector `d` (11) | Place spot per colour | Whenever you like |
| `sim2hw/ik_pose_points.json` | IK server | Recently solved poses, reused as IK starting points | Never by hand |

Back up `sim2hw/cv_pick_corrections.json` and `sim2hw/cv_color_places.json`
before experiments; they hold your teaching.

---

## 16. Daily checklist

**Start**

1. Power the arm, plug in USB and the camera. Check nothing has moved: board,
   camera, arm base.
2. `.\run.ps1 -Port COM6`.
3. Watch the IK server and bridge windows start without errors.
4. In the detector: no "Board/camera moved" warning; objects are listed.
5. Press HOME (`h`) and check the arm moves normally.
6. Do one test pick with `p` then `a`. If it misses, teach a spot or two
   ([13.1](#131-teaching-a-spot-recommended-way-aim)).

**Stop**

1. Press HOME (`h`) so the arm rests in its home pose.
2. Quit the detector (`q` or Esc), then close the bridge and IK server
   windows.
3. Switch off the arm's power.

---

## 17. When to redo what

| After … | Redo |
|---|---|
| Replacing a servo or remounting a horn | 4.2 (new servo only), 4.3–4.5, 6, 8, then DELETE OLD SPOTS and teach |
| The arm misses badly everywhere | Check 4.4. If the ranges look fine, 6.1 |
| Moving the camera | 7 (the detector warns "board/camera moved") |
| Moving the board or the arm base | 7, 8, then teach (old spots are ignored automatically; DELETE OLD SPOTS clears them) |
| Changing the gripper or its jaws | 6.2, then teach |
| A new session with nothing moved | Nothing |

---

## 18. Troubleshooting

### Setup and startup

| Symptom | Cause / fix |
|---|---|
| `run.ps1` / `setup.ps1` will not run | `powershell -ExecutionPolicy Bypass -File .\run.ps1 -Port COM6` |
| "COM6 is not among the serial ports" | Wrong port or cable: `lerobot-find-port` |
| Bridge cannot open the port | Another program has it open (a LeRobot script, another bridge). Close it |
| IK server crashes with `UnicodeEncodeError` | Started by hand without UTF-8: `$env:PYTHONIOENCODING = "utf-8"` first (`run.ps1` does this) |
| `No module named 'mujoco'` | Wrong Python. Use `.venv\Scripts\python.exe` or rerun `.\setup.ps1` |
| Detector window never opens | OpenCV without windows: `.\setup.ps1` again, or `pip install --force-reinstall opencv-python` in `.venv` |
| Black image or wrong camera | `-Camera 0` (or another index), `--backend msmf` / `any`, close other camera apps |

### Arm motion

| Symptom | Cause / fix |
|---|---|
| Arm stops high above the object when told to go to the board | Joint calibration wrong. Check 4.4: a joint near 359° wrapped. Recalibrate |
| Arm goes to the wrong place everywhere | Calibration saved without `--robot.id=follower` (`None.json`), or the bridge was not restarted after calibrating |
| "Target is too close to the base" / red outline | The object is within 8–8.5 cm of the base axis. Move it further out |
| "… is out of the arm's reach: the closest pose misses it by N mm" | The target is just past the arm's reach (often beside the base). Move it a few cm further from the base or more in front of the arm |
| Preflight "model predicts shoulder/gripper contact" | Usually fine on the real arm. Keep a hand near KILL the first time |
| Arm vibrates while still | `-BridgeArgs "--p-gain 12"` |
| Arm sags or lags | `-BridgeArgs "--p-gain 20"` |
| Arm jitters only while holding something | `-BridgeArgs "--log-health 1"`. If the gripper draws ~1 A and voltage dips, lower `--gripper-torque-limit` |
| Objects slip | `--grip-squeeze-deg 8` or `--gripper-torque-limit 650` |
| "Nothing was gripped" on a real grab | The pick missed: teach the spot. Or re-record the gripper close position (6.2) with the jaws fully closed on nothing. Objects under 5 mm thick look empty |

### Detection

| Symptom | Cause / fix |
|---|---|
| "Board/camera moved" warning | Press `c` and click the corners again, starting at the same corner |
| "Height calibration disagrees with the corners by N cm" | `--recalibrate-height` |
| "Overexposed" warning, pale objects missing | `-DetectorArgs "--exposure -7"` |
| Object not listed | Its centre must be well inside the board, it must be still for a few frames and 1–15 cm across. White objects need `--white-objects`. The Status line "Noise filtered" says why contours were rejected |
| Shadows or glare listed as objects | Raise `--confirm-frames`; avoid `--white-objects` |
| Wrong colour name | Check the light and exposure; dull objects are named by tint ("dark green") |

### Detector controls

| Symptom | Cause / fix |
|---|---|
| Keys do nothing | Click the detector window first to give it focus |
| `a`, `f`, `d`, `b` or clicks do nothing | Detector is in preview mode. Start with `-Port` (pick/place mode) |
| Clicking the image does nothing | Press `d` (place spot) or `b` (stacker) first, then click. A plain click only shows a hint |
| Picks miss by the same amount everywhere | Nudge in ALL mode (13.5), or teach spots |
| Picks miss differently across the board | Teach 4–6 spots spread over the board |
| Panel shows many ignored spots | The setup moved since they were taught: Shift+`G` or DELETE OLD SPOTS |

---

## 19. Safety

- **KILL / STOP** (`s` or the red button) is a software stop: the arm holds
  its pose and a held object stays gripped. Keep the arm's power switch within
  reach as the real emergency stop.
- Keep hands out of the workspace while the arm moves. Remove objects or reach
  in only after KILL or when the arm is at HOME.
- Every target is checked before the arm moves, but the check does not know
  about objects outside the board, cables or your hands.
- For the first runs after any calibration, use `-DetectorArgs
  "--move-duration 6"` and watch the first pick in each area of the board.
- Do not leave the arm holding an object under load for long periods; the
  gripper servo heats up.
