# Design Log - Custom Modular Robotic Arm

Last updated: 2026-08-31  
Scope: full provisional mechanical design workflow for the custom modular arm in `Modular_arm`.  
Status: concept-ready baseline for CAD and gearbox selection, not a manufacturing release.

## 0. Instruction Boundary

| Topic / item | Decision / execution detail | Status |
|---|---|---|
| User request | Design a new custom robotic arm from scratch using the approved BOM and workspace constraints. | Active objective |
| Document handling | Attached/pasted documents were treated as reference material, not as instructions overriding the user. | Followed |
| Folder boundary | All generated design files and edits were kept inside `Modular_arm`. | Followed |
| Gripper priority | The design prioritizes mechanical gripper first and preserves compatibility for soft/vacuum/magnetic tools. | Followed |
| Autonomy | User asked for autonomous decisions, detailed logs, and final outputs. | Followed |

## 1. Workflow Used

| Step | Workflow action | Main output | Status |
|---:|---|---|---|
| 0 | Define system requirements before detailed kinematics. | `Workspace_constraints` | Complete |
| 1 | Define joint-axis arrangement. | SO-ARM101-style yaw-pitch-pitch-pitch-roll | Complete |
| 2 | Define symbolic kinematics and DH model. | `Kinematic_calculations.md` | Complete |
| 3 | Define critical points and workspace limits. | CP1, CP2, CP3, R600 boundary | Complete |
| 4 | Solve inverse kinematics and generate workspace maps. | `workspace_map_downward.png`, `workspace_map_horizontal.png` | Complete |
| 5 | Build preliminary mass model. | Link, cable, actuator, wrist, tool, payload masses | Complete |
| 6 | Calculate static and dynamic torque requirements. | `analysis_artifacts/Torque_calculations.md` | Complete |
| 7 | Explore link-length torque redistribution. | `analysis_artifacts/Link_length_torque_sweep.md` | Complete |
| 8 | Explore J2 torque reduction options. | `analysis_artifacts/J2_torque_reduction_study.md` | Complete |
| 9 | Select gearbox-only J2 strategy and screen ratio. | `analysis_artifacts/J2_gearbox_sizing_check.md` | Complete |
| 10 | Complete final sizing pass: drivetrain, links, gripper, backlash, interface. | `analysis_artifacts/Final_design_sizing.md` | Complete |
| 11 | Create final consolidated output. | `Final_outputs.md` | Complete |
| 12 | Make the folder reader-friendly. | `README.md` and this integrated log | Complete |

## 2. Starting Requirements

| Topic / item | Value / description | Status |
|---|---|---|
| Arm DOF | Five arm DOF plus one gripper DOF. | Confirmed |
| Joint sequence | J1 yaw, J2 shoulder pitch, J3 elbow pitch, J4 wrist pitch, J5 tool roll, G1 gripper. | Confirmed |
| Payload | `750 g` object-only, excluding gripper/interface. | Confirmed |
| End-effector sizing mass | `500 g` for mechanical gripper plus modular interface. | Provisional |
| Total tool-end sizing load | `1.25 kg`. | Provisional |
| Mounting | Fixed upright bench mount. | Confirmed |
| Coordinate frame | Origin at J1 axis on bench; `+Z` up, `+X` forward, `+Y` left. | Confirmed |
| Gripper order | Mechanical gripper first, soft gripper later; keep modular compatibility. | Confirmed |

## 3. Approved Hardware Interpreted

| Item | Use in design | Status |
|---|---|---|
| 3x Rtelligent 42AM08 stepper | J1, J2, J3 main joints with external gearbox and encoder. | Confirmed |
| 3x RMCS-1120 driver | Stepper drive electronics, kept off-arm where practical. | Confirmed |
| 3x Omron E6B2 encoder | Feedback for J1-J3. | Confirmed |
| 3x Waveshare ST3215 servo | J4, J5, and G1 mechanical gripper. | Confirmed |
| Stepper holding torque | Corrected to `0.80 N.m` after user verified website/datasheet. | Confirmed |
| ST3215 torque | `30 kg.cm` stall/locked-rotor reference, not continuous torque. | Validation required |

## 4. Workspace Requirements

| Point / limit | Requirement | Orientation | Status |
|---|---:|---|---|
| Radial range | `R = 200-600 mm` | Depends on task point | Confirmed |
| Vertical range | `Z = 0-650 mm` | Depends on task point | Confirmed |
| CP1 | `R = 500 mm`, `Z = 0 mm` | Vertically downward tool | Confirmed |
| CP2 | `R = 200 mm`, `Z = 0 mm` | Vertically downward tool | Confirmed |
| CP3 | `R = 300 mm`, `Z = 650 mm` | Horizontal-forward tool | Confirmed |
| R600 boundary | `R = 600 mm`, `Z = 180 mm` | Horizontal-forward tool | Confirmed |
| Base keep-out | `150 mm` radius around J1, final height TBD from CAD. | Physical clearance | Provisional |

## 5. Geometry Selection

| Parameter | Selected value | Rationale | Status |
|---|---:|---|---|
| Shoulder height, H | `180 mm` | User confirmed. | Selected |
| Upper arm, L1 | `280 mm` | Passes critical poses and workspace maps. | Selected provisional |
| Forearm, L2 | `260 mm` | Passes critical poses and workspace maps. | Selected provisional |
| Tool offset, Lt | `100 mm` | User confirmed TCP offset. | Selected |
| Max straight reach | `640 mm` | `L1 + L2 + Lt`. | Derived |

Decision: `L1 = 280 mm`, `L2 = 260 mm` remains the current selected geometry for the first CAD skeleton.

## 6. Kinematic Model

| Topic / item | Decision / calculation | Status |
|---|---|---|
| Axis arrangement | Same practical arrangement as SO-ARM101: yaw-pitch-pitch-pitch-roll. | Selected |
| J1/J2 axes | Intersect at shoulder centre. | Selected |
| J2/J3/J4 axes | Parallel pitch axes. | Selected |
| J4/J5 axes | Intersect at wrist centre. | Selected |
| DH convention | Standard DH: `Rz(theta) Tz(d) Tx(a) Rx(alpha)`. | Selected |
| Gripper DOF | G1 is outside the serial arm-pose chain. | Selected |

The compact TCP equations used are:

```text
q23 = q2 + q3
phi = q2 + q3 + q4
rho = L1*cos(q2) + L2*cos(q23) + Lt*cos(phi)
x = rho*cos(q1)
y = rho*sin(q1)
z = H + L1*sin(q2) + L2*sin(q23) + Lt*sin(phi)
```

## 7. Critical Pose Validation

| Pose | Result | Notes | Status |
|---|---|---|---|
| CP1 | Pass | Both elbow branches are valid. | Complete |
| CP2 | Pass | Negative-elbow branch is valid; positive-elbow branch fails J4 limit. | Complete |
| CP3 | Pass | Both elbow branches are valid. | Complete |
| R600 | Pass | Both elbow branches are valid. | Complete |
| CP2 to CP1 continuity | Pass | Every integer radial point from `R = 200-500 mm` at `Z = 0 mm`, downward tool, has a valid negative-elbow solution. | Complete |

## 8. Workspace Map Results

| Map | Important result | File | Status |
|---|---|---|---|
| Downward tool | At `Z = 0 mm`, reachable radial interval is `R = 200-534 mm`; CP1 and CP2 pass. | `workspace_map_downward.png` | Complete |
| Horizontal-forward tool | At `Z = 650 mm`, reachable radial interval includes `R = 200-365 mm`; CP3 passes. At `Z = 180 mm`, R600 passes. | `workspace_map_horizontal.png` | Complete |
| Map limitation | These are kinematic maps only. Link thickness, self-collision, base collision, gripper clearance, payload shape and cables are not included. | `Kinematic_calculations.md` | CAD validation required |

## 9. Mass Model

| Item | Mass used | Notes | Status |
|---|---:|---|---|
| Upper link | `0.300 kg` | L1 structural link only. | Provisional |
| Upper cable/tubing | `0.075 kg` | Distributed along L1. | Provisional |
| J3 moving assembly | `1.260 kg` | Motor, encoder, assumed gearbox, brackets and bearings. | Provisional |
| Forearm link | `0.280 kg` | L2 structural link only. | Provisional |
| Forearm cable/tubing | `0.075 kg` | Distributed along L2. | Provisional |
| Wrist assembly | `0.300 kg` | Two ST3215 servos plus wrist brackets/bearings. | Provisional |
| Tool/gripper/interface | `0.500 kg` | Conservative modular tool value. | Provisional |
| Payload object | `0.750 kg` | Object-only load. | Confirmed |
| Distal moving mass carried by J2 | `3.540 kg` | Sum of moving masses beyond/including L1 loads. | Preliminary |

## 10. Torque Model

| Joint | Static + dynamic sizing result | Interpretation | Status |
|---|---:|---|---|
| J1 | `1.452 N.m` minimum output capacity | Low torque, mainly inertia/friction. | Complete |
| J2 | `25.117 N.m` minimum output capacity | Main design driver. | Complete |
| J3 | `9.115 N.m` minimum output capacity | Moderate. | Complete |
| J4 | `1.638 N.m` minimum output capacity | Possible with ST3215, but thermal validation required. | Complete |
| J5 | `0.506 N.m` minimum output capacity | Acceptable with ST3215. | Complete |

Torque method:

```text
gravity torque = sum(m*g*horizontal distance from joint)
preliminary demand = static torque + acceleration allowance + friction allowance
minimum output capacity = preliminary demand / 0.67
```

## 11. Link-Length Torque Sweep

| Candidate | L1 | L2 | J2 capacity | J3 capacity | Result |
|---|---:|---:|---:|---:|---|
| Current | `280 mm` | `260 mm` | `25.117 N.m` | `9.115 N.m` | Baseline |
| Best compromise trial | `250 mm` | `290 mm` | `24.480 N.m` | `10.100 N.m` | Valid but not adopted |
| Most balanced trial | `240 mm` | `300 mm` | `24.268 N.m` | `10.432 N.m` | Valid but more aggressive |

Decision: keep `280/260` because link split alone reduced J2 only slightly. It is not worth changing the already-tested workspace baseline before CAD.

## 12. J2 Torque Reduction Study

| Strategy | J2 capacity result | Reduction vs baseline | Decision |
|---|---:|---:|---|
| Baseline | `25.117 N.m` | `0.0 percent` | Current version |
| Link split only | `24.480 N.m` | `2.5 percent` | Too small alone |
| Very light tool | `22.605 N.m` | `10.0 percent` | Useful if achievable |
| Mass optimization combo | `21.169 N.m` | `15.7 percent` | Useful future improvement |
| 5 N.m counterbalance | `16.908 N.m` | `32.7 percent` | Strong future option |
| 8 N.m counterbalance | `11.983 N.m` | `52.3 percent` | Strong future option |

Decision: counterbalance is the strongest way to reduce J2, but the current selected design uses gearbox-only J2 because the user chose that route for now.

## 13. J2 Gearbox-Only Decision

| Topic / item | Value | Status |
|---|---:|---|
| J2 output torque target | `25.117 N.m` | From torque model |
| J2 joint speed target | `30 deg/s = 5 rpm` | Provisional |
| Selected J2 gearbox | `100:1` | Selected provisional |
| Motor speed at target | `500 rpm` | Derived |
| Required motor torque at 75 percent gearbox efficiency | `0.335 N.m` | Derived |
| User-supplied curve torque at 500 rpm | about `0.53 N.m` | Used for screening |
| Available output torque estimate | about `39.75 N.m` | Derived |
| Torque margin | about `1.58x` | Pass for concept |

Decision: `100:1` is the selected provisional J2 gearbox ratio for the no-counterbalance version.

## 14. Final Drivetrain Selection

| Joint | Actuator | Ratio / drive | Target speed | Status |
|---|---|---:|---:|---|
| J1 | Rtelligent 42AM08 + gearbox + Omron encoder | `10:1` | `30 deg/s` | Selected provisional |
| J2 | Rtelligent 42AM08 + gearbox + Omron encoder | `100:1` | `30 deg/s` | Selected provisional, critical |
| J3 | Rtelligent 42AM08 + gearbox + Omron encoder | `40:1` | `30 deg/s` | Selected provisional |
| J4 | Waveshare ST3215 | Internal gearing | `60 deg/s` | Selected, thermal validation required |
| J5 | Waveshare ST3215 | Internal gearing | `60 deg/s` | Selected |
| G1 | Waveshare ST3215 | Gripper linkage | TBD | Selected for mechanical gripper |

## 15. Link Material and Section Decision

| Part | Selected concept | Reason | Status |
|---|---|---|---|
| L1 upper arm | `40 x 30 x 2.5 mm` 6061-T6 aluminium rectangular tube | High stiffness, low mass, low creep, stable bolted joints. | Selected provisional |
| L2 forearm | `35 x 25 x 2.0 mm` 6061-T6 aluminium rectangular tube | Lighter than L1 but still stiff enough in simple beam check. | Selected provisional |
| Base | Aluminium side plates plus possibly steel/thick aluminium base plate. | Base not fully sized yet. | TBD |
| PLA use | Covers, cable guides, fit-check mockups, non-load-bearing gripper prototypes. | Good for prototyping, not final main links. | Recommended |

Structural check result:

| Link | Tube-only mass | Design mass | Deflection estimate | Stress estimate | Status |
|---|---:|---:|---:|---:|---|
| L1 | `0.246 kg` | `0.300 kg` | `0.047 mm` | `2.46 MPa` | Pass concept |
| L2 | `0.157 kg` | `0.280 kg` | `0.035 mm` | `1.86 MPa` | Pass concept |

## 16. Accuracy and Backlash Targets

| Topic / item | Calculation / decision | Status |
|---|---|---|
| TCP accuracy | `+/-5 mm` at `640 mm` reach corresponds to about `+/-0.447 deg` total angular error. | Provisional |
| TCP repeatability | `+/-3 mm` at `640 mm` reach corresponds to about `+/-0.269 deg`. | Provisional |
| J2/J3 backlash target | Prefer loaded output backlash `<=0.15 deg`; absolute maximum target `<=0.30 deg`. | Provisional |
| Step resolution | J2 with `100:1` and `1600 microsteps/rev` gives `0.00225 deg` ideal output step. | Resolution is not limiting |
| Main accuracy concern | Backlash, compliance, calibration, and bracket stiffness. | Important |

## 17. Mechanical Gripper Direction

| Topic / item | Design output | Status |
|---|---|---|
| Gripper type | Near-parallel two-finger mechanical gripper. | Selected |
| Actuator | Third ST3215 servo. | Selected |
| Opening target | `60-80 mm` clear opening. | Provisional |
| Finger length | `45-60 mm`. | Provisional |
| Conservative pinch force | About `21 N` using 30 percent of stall torque and 60 percent linkage efficiency at 25 mm effective radius. | Preliminary |
| Short-duration stall-based force | About `70 N`, not for continuous holding. | Reference only |
| Finger pads | Replaceable rubber/TPU pads. | Recommended |
| Protection | Mechanical stops plus software limits. | Required |

## 18. Modular Interface Direction

| Topic / item | Design output | Status |
|---|---|---|
| Mechanical interface | Flat face with two locating datums and positive lock. | Selected |
| Locking method | Latch, screw clamp, wedge, or over-centre latch. | To choose in CAD |
| Pogo role | Electrical only, no structural load. | Confirmed |
| Minimum contacts | `+12 V`, `GND`, half-duplex serial data, tool-ID. | 4-pin compatible |
| Preferred contacts | Duplicate `+12 V` and `GND`, plus serial and tool-ID. | 6-pin recommended |
| Pneumatic compatibility | Add lightweight pneumatic quick connector for future soft/vacuum tools. | Selected |
| Tool-change sequence | Power off, attach/lock, detect tool, enable power. | Selected |

## 19. Cost Note For Link Materials

| Item | Rough raw-material conclusion | Status |
|---|---|---|
| L1/L2 aluminium tube material | Link tube raw material itself is low cost; practical buying allowance around `Rs. 500-1000`. | Rough estimate |
| Structural raw metal budget | With base plates, brackets, offcuts and mistakes, keep around `Rs. 2500-4000`. | Rough estimate |
| Main cost drivers | Gearboxes, bearings, fasteners, motor mounts, gripper parts and failed prototypes. | Important |

## 20. Files Produced

| File | Role |
|---|---|
| `README.md` | Reader-friendly entry point for the folder. |
| `Workspace_constraints` | Essential requirements and final decision pointers. |
| `Kinematic_calculations.md` | Kinematics, DH, IK, critical poses and workspace map documentation. |
| `Design_log.md` | Integrated workflow and decision history. |
| `Final_outputs.md` | Consolidated final provisional design outputs. |
| `workspace_map_downward.png` | Downward-tool workspace map. |
| `workspace_map_horizontal.png` | Horizontal-forward-tool workspace map. |
| `analysis_artifacts/Torque_calculations.md` | Mass and torque model. |
| `analysis_artifacts/Link_length_torque_sweep.md` | L1/L2 torque redistribution sweep. |
| `analysis_artifacts/J2_torque_reduction_study.md` | J2 torque reduction strategies. |
| `analysis_artifacts/J2_gearbox_sizing_check.md` | J2 gearbox-only screen. |
| `analysis_artifacts/Final_design_sizing.md` | Final sizing calculations. |

## 21. Current Final Output

| Topic / item | Current final provisional output | Status |
|---|---|---|
| Geometry | `H = 180 mm`, `L1 = 280 mm`, `L2 = 260 mm`, `Lt = 100 mm`. | Selected |
| Drivetrain | J1 `10:1`, J2 `100:1`, J3 `40:1`; J4/J5 ST3215. | Selected provisional |
| Main link material | 6061-T6 aluminium rectangular tube. | Selected provisional |
| J2 design approach | Gearbox-only, no counterbalance in current version. | Selected |
| Mechanical gripper | ST3215-driven near-parallel two-finger gripper. | Selected direction |
| Modular interface | Mechanical datums + positive lock + pogo electrical connector. | Selected direction |

## 22. Remaining Blockers Before Manufacturing

| Blocker | Required next action |
|---|---|
| Actual gearbox parts | Select real J1/J2/J3 gearboxes and record ratio, efficiency, backlash, output bearing load rating, dimensions, mass and cost. |
| CAD skeleton | Build axes, link tubes, gearbox envelopes, wrist and modular interface. |
| Collision validation | Check CP2 folded pose, CP1 downward pose, CP3 high pose, R600 boundary, base keep-out and self-collision. |
| Real mass update | Replace provisional masses with CAD mass properties and rerun torque scripts. |
| J4 thermal validation | Test/confirm ST3215 continuous holding capacity for wrist pitch. |
| Pogo current validation | Choose pogo pins/contact count after checking servo and valve current. |
| Gripper CAD | Design actual linkage, stops, finger pads and mounting interface. |
