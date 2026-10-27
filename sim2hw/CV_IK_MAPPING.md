# CV to IK calibration points

The IK server records named board coordinates paired with the robot model's
end-effector XYZ in the IK world frame. The CV detector fits a board-to-robot
plane transform from those points and sends preview-only targets. This does not
command arm motion.

## Record board points

1. Start `python sim2hw/ik_server.py` and `python sim2hw/hw_bridge.py --port COM4`
   (replace `COM4` with the arm's port). Connect to both in the coordinate UI.
2. Choose at least three reachable points on the board. They do not need to be
   corners, and the wrist does not need to point straight down. Wrist roll may
   differ between points. At each point, place the same gripper reference point
   (`gripperframe`) over the measured board X/Y. Keep the points spread across
   the reachable area and non-collinear; points along one line cannot define a
   2D board mapping.
3. For each point, position the model's `gripperframe` reference at the measured
   board X/Y on the board surface. In **CV Board Calibration Points**, enter a
   name and that board X/Y in cm, then click **Record Current Arm Point**. If the
   arm is not connected, records come from the simulation and will not calibrate
   the physical arm.

The server writes/replaces each named point in `sim2hw/cv_robot_points.json`.
The file stores board XY in cm, robot XYZ in metres, the coordinate-frame name,
the source (hardware joint FK or simulation), and a timestamp. The hardware XYZ
is calculated from the hardware's live joint angles using the server's MuJoCo
model; it is not an independent physical Cartesian sensor measurement. Wrist
roll can vary because the server computes XYZ from the full current joint pose.

## Calibrate the 4 cm reference plane

The detector uses a 6 x 6 cm square whose top face is 4 cm above the board.
During first run (or with `--recalibrate-height`), it asks you to place the
square snug into each of the four board corners, with both square edges flush
against the board edges. You do not need to find or mark the center coordinates.
For a 21.3 x 28.3 cm board, the derived centers are (3, 3), (18.3, 3),
(18.3, 25.3), and (3, 25.3) cm. Keep the square's top face at 4 cm above the
board. For each placement, press Enter/Space when still, then click the four
corners as prompted. Repeat at all four corners. These captures fit the
raised-plane image-to-board mapping and account for placement angle automatically.

The calibration is saved in `workspace_height_calibration.json`. The old
single-position 8 x 6 cm calibration is replaced because its dimensions no
longer match.

Copy `cv_ik_mapping.example.json` to `cv_ik_mapping.json` if needed. The
`robot_points_file` should point to the generated point file, and
`tool_contact_offset_m` can stay at 0 for an initial reachability preview.
## Preview

With IK server and camera running, launch:

```powershell
python CV-seg/Robotic_Arm-main/object_detector_standalone.py --ik-preview --ik-mapping sim2hw/cv_ik_mapping.json
```

Select an object with **1–9**, then press **p**. The terminal prints target XYZ
and the IK preflight result. This checks reachability only; it does not check
collision clearance or grasp success.

## Pick one object and place it near Corner 2

With the arm clear and the IK server and hardware bridge online, run:

```powershell
python CV-seg/Robotic_Arm-main/object_detector_standalone.py --ik-pick-place --ik-mapping sim2hw/cv_ik_mapping.json
```

Select one visible object with **1–9**. Press **p** to preview both targets with
the gripper pointing down; press **a** to run one pick-and-place cycle. The
program preflights both targets before moving. It keeps the target centered in
X/Y, opens the gripper, and grasps near the object's lower side at 1 cm above
the board by default. It then releases near Corner 2 at the same height. The 4 cm
object height remains the measured object/reference height; use `--grasp-height`
to tune the lower-side grip height.

Use the on-screen feedback keys to correct either target between cycles. Press
**m** to switch the nudge mode between **PICK** and **DROP**. Then press **l/j**
for +X/−X, **i/k** for +Y/−Y, or **u/o** for +Z/−Z. The default steps are 5 mm
for X/Y and 2 mm for Z; set them with `--feedback-step-xy-mm` and
`--feedback-step-z-mm`. Press **0** to clear the offset for the active mode.
The displayed pick and drop offsets are in the robot's IK XYZ frame and apply
to the next preview or cycle. Use **p** after nudging to check the corrected
pick and drop targets before pressing **a**.

### Teach corrections (fixes the arm missing the object)

The recorded board points are converted to robot XYZ through the MuJoCo model,
which can be several cm off the real arm, and they were recorded in other wrist
poses than the gripper-down pick pose. Picks therefore miss by an amount that
changes across the board. Teach the real offsets instead:

1. Put an object somewhere on the board, select it, and press **a** (or **p**).
2. Nudge the **pick** (l/j/i/k/u/o) and repeat until the gripper closes centred
   on the object.
3. Press **g**. The nudged target is saved as a correction for that board spot
   in `sim2hw/cv_pick_corrections.json`, and the nudge resets to zero. If the
   pick was already centred, press **g** without nudging: that confirms the
   spot as correct, which teaches the model just as much.

To shift **everything** at once (every pick and drop, anywhere on the board),
press **m** until the nudge mode reads **ALL**, then use the same nudge keys.
The shift applies immediately, is saved in the corrections file (so it lasts
across sessions) and shows as "All targets" on the panel; **0** in ALL mode
resets it. Taught spots are stored without it, so changing it never counts
twice.

Press **t** to switch learning mode off for normal use: everything learned is
still applied, but **g**/**Shift+G** are disabled and the teach-here marker is
hidden. Start the detector with `--learning-mode off` to begin that way.
4. Repeat at 4–6 spots spread over the board (near the base, far side, both
   sides). Press **m** and do the same for the Corner 2 **drop** once.

How the taught spots are used:

- **3+ consistent spots → learned arm model.** The board is fitted as a rigid,
  true-size rectangle in the robot frame, plus small pan-gain and reach errors
  around the arm base (how joint calibration errors show up). Each taught spot
  then replays exactly. The panel shows the typical miss at a *new* spot,
  estimated by leaving each spot out in turn.
- **Setup changes are detected.** Spots that disagree with the newest one by
  more than 2.5 cm under a rigid fit (taught before the board, camera or arm
  moved) are ignored and counted on the panel; **Shift+G** deletes them. Spots
  from the current teaching session count more than older sessions.
- **Fewer spots, or when it predicts better:** the corner map plus a blend of
  the consistent corrections (about 1.5 cm radius) is used instead.
- Spots where the object overhangs the board edge count less, because its
  visible centre is biased.
- The yellow **teach here** marker shows the board area farthest from any
  taught spot. Spread spots over the board: the model extrapolates poorly into
  areas with no spots.

Re-teaching within 1.5 cm of a saved spot replaces it. Each gripper side
(**w**) learns separately. Start over with `--clear-ik-corrections` or by
deleting the file.

Corner 2 is the board's (21.3, 0) cm corner. To keep each detected object fully
on the board, its drop center is inset from both edges by its half-diagonal plus
0.5 cm. The place height is computed from the recorded arm-point plane plus the
4 cm object height. Each press of **a** handles one object; select another and
press **a** again for another cycle. This triggers real arm motion, so use it
only after confirming the displayed object selection and clearing the arm path.
