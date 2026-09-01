# Modular Arm Design Package

This folder contains the provisional mechanical design package for the custom modular robotic arm.

The current design is a concept-ready baseline for CAD. It is not a manufacturing release yet because gearbox selection, CAD collision checks, actual mass properties, and hardware thermal validation are still pending.

## Quick Answer

| Topic / item | Current output | Status |
|---|---|---|
| Arm architecture | 5 arm DOF + 1 gripper DOF | Selected |
| Joint layout | J1 yaw, J2 shoulder pitch, J3 elbow pitch, J4 wrist pitch, J5 wrist roll, G1 gripper | Selected |
| Shoulder height | `H = 180 mm` | Selected |
| Upper arm | `L1 = 280 mm` | Provisional |
| Forearm | `L2 = 260 mm` | Provisional |
| Tool offset | `Lt = 100 mm` | Selected |
| Payload | `750 g` object-only | Confirmed |
| Tool/gripper/interface mass | `500 g` | Provisional |
| J1 gearbox | `10:1` | Provisional |
| J2 gearbox | `100:1` | Provisional, critical |
| J3 gearbox | `40:1` | Provisional |
| Counterbalance | Not used in current selected version | User-directed |
| L1 tube concept | `40 x 30 x 2.5 mm` 6061-T6 aluminium rectangular tube | Provisional |
| L2 tube concept | `35 x 25 x 2.0 mm` 6061-T6 aluminium rectangular tube | Provisional |

## Where To Start

Read these files in this order:

| File | What it is for |
|---|---|
| [`Final_outputs.md`](Final_outputs.md) | The shortest complete summary of the current design output. |
| [`Design_log.md`](Design_log.md) | Full start-to-finish workflow and decision history. |
| [`Workspace_constraints`](Workspace_constraints) | Essential requirements, constraints, selected values, and validation status. |
| [`Kinematic_calculations.md`](Kinematic_calculations.md) | DH model, forward kinematics, inverse kinematics, critical poses, and workspace map results. |
| [`analysis_artifacts/Final_design_sizing.md`](analysis_artifacts/Final_design_sizing.md) | Final drivetrain, structure, gripper, backlash, and interface calculations. |

## Folder Map

| Item | Purpose |
|---|---|
| `README.md` | Orientation file for this folder. |
| `Final_outputs.md` | Consolidated final provisional outputs. |
| `Design_log.md` | Integrated design workflow log from start to current state. |
| `Workspace_constraints` | Essential constraints and decisions. |
| `Kinematic_calculations.md` | Main kinematics documentation. |
| `workspace_map_downward.png` | Reachable workspace map for vertically downward tool orientation. |
| `workspace_map_horizontal.png` | Reachable workspace map for horizontal-forward tool orientation. |
| `analysis_artifacts/` | Detailed calculations, scripts, CSVs, SVGs, and JSON outputs. |

## Current Workspace Result

| Requirement | Result | Status |
|---|---|---|
| CP1 | `R = 500 mm`, `Z = 0 mm`, downward tool | Pass |
| CP2 | `R = 200 mm`, `Z = 0 mm`, downward tool | Pass |
| CP3 | `R = 300 mm`, `Z = 650 mm`, horizontal-forward tool | Pass |
| R600 boundary | `R = 600 mm`, `Z = 180 mm`, horizontal-forward tool | Pass |
| CP2 to CP1 continuity | Every radial point from `R = 200-500 mm` at `Z = 0 mm`, downward tool | Pass with negative-elbow branch |

The workspace maps are kinematic maps. They do not yet include physical link thickness, base collision, self-collision, cable routing, or gripper/payload clearance.

## Current Torque Result

| Joint | Required minimum output capacity | Current selection |
|---|---:|---|
| J1 base yaw | `1.45 N.m` | 42AM08 + `10:1` gearbox |
| J2 shoulder pitch | `25.12 N.m` | 42AM08 + `100:1` gearbox |
| J3 elbow pitch | `9.12 N.m` | 42AM08 + `40:1` gearbox |
| J4 wrist pitch | `1.64 N.m` | ST3215, thermal validation required |
| J5 wrist roll | `0.51 N.m` | ST3215 |

The J2 `100:1` gearbox choice is based on the current no-counterbalance design. At the target J2 speed of `30 deg/s`, the motor needs to run at about `500 rpm`. Using the supplied 42AM08 torque-speed curve, the concept margin is about `1.58x`.

## Important Design Decisions

| Decision | Reason |
|---|---|
| Keep `L1 = 280 mm`, `L2 = 260 mm` for first CAD pass | It already passes the workspace and critical-pose checks. |
| Do not adopt `L1 = 250 mm`, `L2 = 290 mm` yet | It improves torque distribution, but only slightly reduces J2 torque. |
| Use gearbox-only J2 for current version | User chose to avoid counterbalance for now. |
| Keep aluminium for main links | Aluminium tube gives stiffness and dimensional stability with low mass. PLA is better for mockups/covers/non-load-bearing parts. |
| Use pogo pins for electrical only | The tool interface must carry structure through mechanical datums and locks, not pogo contacts. |

## What Is Still Not Final

| Blocker | Why it matters |
|---|---|
| Actual gearbox selection | Need real ratio, efficiency, backlash, output bearing rating, dimensions, mass, and price. |
| CAD collision checks | Current maps are ideal kinematic checks only. |
| CAD mass properties | Provisional masses must be replaced by measured CAD values. |
| J4 continuous torque | ST3215 stall torque is not the same as continuous usable torque. |
| Pogo current rating | Servo current may require duplicated power/ground contacts or separate power contacts. |
| Gripper linkage | Opening, finger geometry, force, and end stops still need actual CAD design. |

## Reproducible Calculations

Most calculations are reproducible from PowerShell scripts in `analysis_artifacts/`:

| Script | Output |
|---|---|
| `workspace_map_analysis.ps1` | Workspace map CSV/SVG/JSON outputs. |
| `torque_model.ps1` | Mass and torque model summary. |
| `link_length_torque_sweep.ps1` | L1/L2 sweep results. |
| `j2_torque_reduction_study.ps1` | J2 torque reduction comparison. |
| `j2_gearbox_sizing_check.ps1` | J2 gearbox ratio screen. |
| `final_design_sizing.ps1` | Final provisional drivetrain, link, gripper, and interface sizing. |

## Next Practical Step

Choose actual gearboxes for:

| Joint | Target ratio |
|---|---:|
| J1 | `10:1` |
| J2 | `100:1` |
| J3 | `40:1` |

The gearbox comparison should include torque rating, efficiency, backlash, mass, dimensions, output bearing/load capacity, availability, and cost.
