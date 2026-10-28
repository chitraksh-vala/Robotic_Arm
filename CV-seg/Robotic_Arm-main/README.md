# Object detector (vision pick-and-place)

`object_detector_standalone.py` watches the workspace board through a camera,
finds objects, measures them in centimetres, and (with the IK server and
hardware bridge from [`sim2hw/`](../../sim2hw/README.md)) picks a selected
object and places it at the board's Corner 2. It learns from your corrections
so picks get more accurate over time.

## Quick start

From the repository root:

```powershell
.\setup.ps1                 # once
.\run.ps1 -Port COM4        # IK server + bridge + this detector (pick/place mode)
.\run.ps1                   # no real arm: detector + IK previews only
```

Or run it directly, from the repository root, with the IK server (and bridge)
already running:

```powershell
.\.venv\Scripts\python.exe CV-seg\Robotic_Arm-main\object_detector_standalone.py --ik-pick-place
```

| Mode | Flag | Use |
|---|---|---|
| Detect only | (none) | Camera, detection and measurements |
| Preview | `--ik-preview` | Also check whether the arm can reach a selected object (`p`) |
| Pick/place | `--ik-pick-place` | Also run picks (`a`), nudges, learning and the KILL/HOME buttons |

Data files are always read from and written to the repository root and
`sim2hw/`, whatever folder you start from.

## First run: calibration

1. **Board corners.** Click the four inner corners of the taped board, in
   order. The **first click is board (0, 0)**. Corner 1 → 2 is the 21.3 cm side
   (board X), corner 2 → 3 the 28.3 cm side (board Y). Enter accepts, `u` undoes
   a click, `r` restarts. Press `c` at any time to redo this; always click the
   same physical corner first. A reference image is saved alongside, and the
   panel warns if the board or camera later moves by more than 8 px.
2. **Object height plane.** Objects are measured on their top face. Place the
   6 × 6 cm reference square (top face 4 cm high) snug into each board corner as
   prompted (a yellow outline shows where), press Enter, then click the square's
   four top corners. Redo with `--recalibrate-height`. If this calibration was
   made with a different first corner, it is relabelled automatically to match
   the corners.
3. **Board to robot.** Record at least three board points with the arm in the
   coordinate-control page; see [CV_IK_MAPPING.md](../../sim2hw/CV_IK_MAPPING.md).
   This is only the starting map: taught spots (below) refine it.

## Using it

Select an object with `1`–`9`, press `p` to check it can be reached, then `a`
to pick it and place it at Corner 2. The panel shows the IK result, warnings
(moved board, overexposure, self-contact), the objects and the learning state.
Objects too close to the arm are outlined in red.

### Keys

| Key | Action |
|---|---|
| `1`–`9` | Select an object |
| `p` / `a` | Preview / run pick and place |
| `s` or KILL button | Stop the arm now |
| `h` or HOME button | Stop, then go to the home pose |
| `m` | Nudge target: PICK → DROP → ALL |
| `l` / `j`, `i` / `k`, `u` / `o` | Nudge +X/−X, +Y/−Y, +Z/−Z (5 mm XY, 2 mm Z) |
| Shift + those keys | Fine 1 mm nudge |
| `0` | Reset the active nudge |
| `g` | Save the nudged pick/drop as a taught spot; with no nudge it confirms the spot was already right |
| Shift+`G` | Delete taught spots the model ignores as inconsistent |
| `t` | Learning mode on/off |
| `w` | Gripper motor jaw toward / away from the robot base |
| `c` | Recalibrate the board corners |
| `q` / Esc | Quit |

### Teaching the arm (learning mode)

The recorded board points go through a model of the arm that can be a few cm
off the real one, so the first picks may miss. Teach it:

1. Pick an object (`a`). If the gripper is off-centre, nudge the PICK and repeat.
2. When it grips centred, press `g` (with no nudge if it was already right).
3. Repeat at the yellow **teach here** marker, which points to the reachable
   board area farthest from any taught spot, until the panel's expected miss is
   small.

With 3+ consistent spots the detector fits a model shaped like the real error
(the board placed rigidly at its true size, plus the arm's pan and reach
errors) and replays each taught spot exactly. Spots that disagree with the
newest ones, usually taught before the board, camera or arm moved, are ignored
and counted on the panel. Objects overhanging the board edge count less. Each
gripper side (`w`) learns separately.

- **Shift everything:** press `m` until the mode reads ALL, then nudge. Every
  pick and drop moves by that amount, saved across sessions.
- **Learning mode off** (`t`, or `--learning-mode off`): learned spots are still
  used; `g` / Shift+`G` and the marker are disabled.
- Taught spots are saved in `sim2hw/cv_pick_corrections.json`. Start over with
  `--clear-ik-corrections` (or delete the file).

### What the detector does

- **Lighting.** Each frame is white-balanced and brightness-flattened against
  the white board, and thresholds adapt to the measured board, so dim, bright,
  warm, cool and uneven light work.
- **Colours.** Saturated colours by hue. Dull or dark objects are found by being
  darker than the board and named by their average tint ("dark green"), or
  black / gray. White objects only with `--white-objects`.
- **Clean outlines.** Patterned faces are grown to the object's real edge;
  board-coloured cast shadows along the edge are cut away.
- **Noise.** Contours must be solid, thick enough, not along the board border,
  between `--min-object-cm` and `--max-object-cm`, and present for
  `--confirm-frames` frames. Positions are smoothed and labels voted over
  frames, so the list does not flicker.

## Options

| Option | Default | Meaning |
|---|---|---|
| `--camera` | 1 | Camera index |
| `--backend` | dshow | Capture backend (`msmf` or `any` if the image is black) |
| `--width`, `--height` | 1280, 720 | Capture size |
| `--exposure` | auto | Manual exposure (DirectShow ≈ −4 bright … −9 dark); lower if pale objects look pure white |
| `--lock` | off | Try to fix focus and white balance |
| `--board-length`, `--board-width` | 21.3, 28.3 | Board size in cm |
| `--object-height` | 4.0 | Object top-face height in cm |
| `--grasp-height` | 1.0 | Gripper height above the board when grasping (cm) |
| `--move-duration` | 4 | Seconds per arm motion |
| `--feedback-step-xy-mm`, `--feedback-step-z-mm`, `--feedback-fine-step-mm` | 5, 2, 1 | Nudge steps |
| `--learning-mode` | on | Start with learning on or off |
| `--gripper-motor-side` | base | `base`, `away` or `auto` |
| `--ik-url` | http://127.0.0.1:7842 | IK server |
| `--recalibrate`, `--recalibrate-height` | off | Redo the corner / height calibration |
| `--min-object-cm`, `--max-object-cm` | 1, 15 | Accepted object size |
| `--confirm-frames` | 4 | Frames before an object is listed |
| `--roi-inset` | 10 | Keep red/orange/pink detections this far (px) inside the corners, away from the red tape |
| `--white-objects` | off | Also detect white objects (bright glare looks the same) |

## Files

| File | Content |
|---|---|
| `workspace_calibration.json` (+ `_reference.png`) | Board corners and the image for the moved-board check |
| `workspace_height_calibration.json` | 4 cm plane calibration |
| `sim2hw/cv_ik_mapping.json`, `sim2hw/cv_robot_points.json` | Board-to-robot starting map |
| `sim2hw/cv_pick_corrections.json` | Taught spots and the ALL shift |

## Troubleshooting

| Symptom | Try |
|---|---|
| Black or no image | `--camera 0`, or `--backend msmf` / `any`; close other apps using the camera |
| "Board/camera moved" warning | Press `c` and recalibrate |
| "Overexposed" warning, pale objects missing | `--exposure -7` (lower = darker) |
| Shadows or glare listed as objects | Raise `--confirm-frames`; avoid `--white-objects` |
| Picks miss | Teach spots (`g`); check the panel's ignored-spots count after moving anything |
| "Too close to the arm" | The object is within 8.5 cm of the arm's base axis, which the arm cannot reach; move it further out |
| "Nothing was gripped" | Nudge the pick onto the object; see the gripper notes in [sim2hw/README.md](../../sim2hw/README.md) |
