# Object detector (vision pick-and-place)

`object_detector_standalone.py` watches the workspace board through a camera,
finds objects, measures them in centimetres, and (with the IK server and
hardware bridge from [`sim2hw/`](../../sim2hw/README.md)) picks a selected
object and places it at the spot set for its colour (Corner 2 by default). It learns from your corrections
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
to pick it and place it at its colour's place spot (see below). The panel shows the IK result, warnings
(moved board, overexposure, self-contact), the objects and the learning state.
Objects too close to the arm are outlined in red.

### Keys

| Key | Action |
|---|---|
| `1`–`9` | Select an object |
| `p` / `a` | Preview / run pick and place |
| `f` | Aim: hover the open gripper over the selected object; every nudge moves the arm there at once. `f` again stops |
| `s` or KILL button | Stop the arm now |
| `h` or HOME button | Stop, then go to the home pose |
| `m` | Nudge target: PICK → DROP → ALL |
| `l` / `j`, `i` / `k`, `u` / `o` | Nudge +X/−X, +Y/−Y, +Z/−Z (5 mm XY, 2 mm Z) |
| Shift + those keys | Fine 1 mm nudge |
| `0` | Reset the active nudge |
| `g` | Save the nudged pick/drop as a taught spot; with no nudge it confirms the spot was already right |
| Shift+`G` | Delete taught spots the model ignores as inconsistent |
| DELETE OLD SPOTS button | Delete spots from earlier sessions, keep the current one (click twice within 4 s) |
| `t` | Learning mode on/off |
| `w` | Gripper motor jaw toward / away from the robot base |
| `d`, then click the image | Set the place spot (on or off the board) for the selected object's colour; with nothing selected, for all other colours. `d` again cancels |
| Shift+`D` | Clear that place spot |
| `b`, then click the image | Stacker: stack every visible object at the click. `b` while it runs stops after the current object |
| `c` | Recalibrate the board corners |
| `q` / Esc | Quit |

### Place spots by colour

Each colour can have its own place spot. Objects of any other colour go to
the default spot. If no default is set, they go to Corner 2, inset so the
whole object stays on the board.

To add a spot for a colour:

1. Put an object of that colour on the board and select it (`1`–`9`).
2. Press `d` and click where it should go. The click is the centre of the
   placed object.
3. The detector checks straight away that the arm can reach the spot. If it
   cannot, the status line says why (for example too close to the base, or
   too far round to the side) and how to fix it; press `d` and click again.
   Before a pick, `p` checks both the pick and the place spot.

The place spot does not have to be on the board: click anywhere in the camera
image, for example on a bin beside it. Objects are still only picked from
inside the board. Off the board, the target comes from extending the
board-to-robot map past its edges, so it is less accurate and assumes the
surface is level with the board. Check it with `p`, and if the drop lands off
or too low, nudge DROP (`m`) and teach it with `g`.

To set the default spot for other colours, press `d` with no object selected
and click. Shift+`D` clears the spot (with an object selected: its colour;
with none: the default). The spots appear as crosses on the camera image and
in the panel, and each object in the list shows where it will go
(`-> red spot`).

The spots are saved in `sim2hw/cv_color_places.json` as board centimetres, so
you can also edit them by hand:

```json
{
  "default": null,
  "colors": {
    "red": [4.0, 4.0],
    "blue": [17.0, 24.0]
  }
}
```

Colour names are the detector's labels: `red`, `orange`, `yellow`, `lime`,
`green`, `teal`, `cyan`, `blue`, `indigo`, `purple`, `magenta`, `pink`,
`black`, `gray`, `white`. A dull object labelled `dark green` uses the
`dark green` spot if there is one, otherwise the `green` spot. Objects of the
same colour all go to the same spot, so move a placed object away before placing another of that
colour, or they land on top of each other.

### Stacker mode

Stacks every visible object, one on top of another, at a point you click.

1. Make sure the objects are visible and listed in the panel.
2. Press `b`, then click where the stack should be. The click can be on or off
   the board. Click an object to stack the others on top of it: that object
   stays where it is.
3. The arm first checks that every pick and the top of the stack are
   reachable, so an unreachable object stops it before anything moves. Then
   it moves the objects nearest the stack first. The panel shows progress and
   a yellow `stack` circle marks the spot.

Each level is `--object-height` (4 cm) higher than the one below. Objects
above the first are released `--stack-clearance` (0.5 cm) above the one below
so they are not pushed into it. All objects must be the same height.

Press `b` while stacking to stop after the object being moved, or `s` / KILL
to stop at once. Objects too close to the arm base are skipped. The object
positions are taken when you click, so do not move objects while it runs. The
pick and drop nudges (`m`) apply to every pick and every stack level.

### Teaching the arm (learning mode)

The recorded board points go through a model of the arm that can be a few cm
off the real one, so the first picks may miss. Teach it:

1. Put an object at the yellow **teach here** marker and select it (`1`–`9`).
   The marker spreads the spots over the board: it picks the quarter of the
   reachable board with the fewest taught spots, and the emptiest point in
   it, at least 12 cm from the arm's base (closer in, the gripper can hit the
   shoulder).
2. Press `f`. The arm hovers the open gripper 1 cm above the object
   (`--aim-clearance`).
3. Nudge (`l`/`j`, `i`/`k`, Shift for 1 mm steps) until the jaws are centred
   over the object. The arm moves after every nudge, about 5 s per move, so
   you see the result straight away. Quick nudges are combined into one move.
4. Press `g` to save the spot (with no nudge if it was already right).
5. Repeat at the next marker until the panel's expected miss is small.

You can still teach the old way: run a pick (`a`), nudge, run it again, and
press `g` when it grips centred. Aiming skips the full pick-and-place cycle
for every nudge. To teach a drop spot, press `m` until DROP is active, then
`f` aims over the selected object's place spot.

Each spot counts for the whole board. From two spots the learner corrects the
board's shift and rotation. From three it fits the full arm model. Spots from
earlier sessions (more than an hour before the newest one) count at 10% in
the fit, so after a recalibration or a small move of the board the new spots
take over within a few teaches. Replaying the Oct 31 session on top of the
older spots, the median miss on not-yet-taught spots fell to about 2 mm after
6 new spots (6 mm before this change).

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
- **Delete old learning:** click DELETE OLD SPOTS in the panel, then click it
  again within 4 s to confirm. It deletes every spot taught before the current
  session (a session ends at a gap of more than an hour) and keeps the current
  session's spots. Use it after a recalibration or after moving the board. The
  previous file is saved as `sim2hw/cv_pick_corrections.json.bak`; copy it back
  over the file and restart the detector to undo.
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
| `--aim-clearance` | 1.0 | Aim (`f`): hover height above the object top (cm) |
| `--stack-clearance` | 0.5 | Stacker: release height above the object below (cm) |
| `--color-places` | `sim2hw/cv_color_places.json` | Place spot per colour (`d` sets one) |
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
| `sim2hw/cv_color_places.json` | Place spot per colour and the default spot |

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
