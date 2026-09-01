# Final Outputs - Provisional Custom Modular Robotic Arm

Status: provisional design output, ready for CAD concept modelling and gearbox procurement comparison. This is not a manufacturing release.

## 1. Final selected baseline

| Topic / item | Final provisional output | Status |
|---|---|---|
| Architecture | 5 arm DOF plus 1 gripper DOF. | Selected |
| Joint arrangement | J1 yaw, J2 shoulder pitch, J3 elbow pitch, J4 wrist pitch, J5 wrist roll, G1 gripper. | Selected |
| Shoulder height | `H = 180 mm` | Selected |
| Upper arm | `L1 = 280 mm` | Selected provisional |
| Forearm | `L2 = 260 mm` | Selected provisional |
| Tool offset | `Lt = 100 mm` from wrist centre to TCP. | Selected |
| Max straight TCP reach | `640 mm` | Derived |
| Payload | `750 g` object-only load. | Confirmed |
| Gripper/interface sizing mass | `500 g` | Provisional |
| Total tool-end sizing load | `1.25 kg` | Provisional |

## 2. Workspace outputs

| Requirement | Result | Status |
|---|---|---|
| CP1 | `R = 500 mm`, `Z = 0 mm`, downward tool passes. | Pass |
| CP2 | `R = 200 mm`, `Z = 0 mm`, downward tool passes. | Pass |
| CP3 | `R = 300 mm`, `Z = 650 mm`, horizontal-forward tool passes. | Pass |
| R600 boundary | `R = 600 mm`, `Z = 180 mm`, horizontal-forward tool passes. | Pass |
| Downward map at bench level | Reachable `R = 200-534 mm` at `Z = 0 mm`. | Pass for CP1/CP2 |
| Horizontal map at top height | Reachable `R = 200-365 mm` at `Z = 650 mm`. | Pass for CP3 |

## 3. Final drivetrain outputs

| Joint | Actuator | Ratio/output | Required capacity | Target speed | Result | Status |
|---|---|---:|---:|---:|---|---|
| J1 | Rtelligent 42AM08 + gearbox + encoder | `10:1` | `1.45 N.m` | `30 deg/s` | Comfortable margin. | Selected |
| J2 | Rtelligent 42AM08 + gearbox + encoder | `100:1` | `25.12 N.m` | `30 deg/s` | Works with supplied curve; gearbox quality is critical. | Selected |
| J3 | Rtelligent 42AM08 + gearbox + encoder | `40:1` | `9.12 N.m` | `30 deg/s` | Comfortable margin. | Selected |
| J4 | ST3215 serial servo | Internal gearing | `1.64 N.m` | `60 deg/s` | Needs thermal/continuous torque validation. | Selected with risk |
| J5 | ST3215 serial servo | Internal gearing | `0.51 N.m` | `60 deg/s` | Acceptable. | Selected |
| G1 | ST3215 serial servo | Gripper linkage | TBD | TBD | Use for mechanical gripper. | Selected |

## 4. J2 gearbox conclusion

| Topic / item | Output |
|---|---|
| Selected current J2 approach | Gearbox only, no counterbalance. |
| Selected ratio | `100:1` |
| Joint speed target | `30 deg/s = 5 rpm` |
| Motor speed needed | `500 rpm` |
| Required motor torque at 75 percent gearbox efficiency | `0.335 N.m` |
| Approx available motor torque from supplied curve | `0.53 N.m` at `500 rpm` |
| Estimated available output torque | `39.75 N.m` |
| Torque margin | `1.58x` |
| Main warning | Gearbox backlash and output bearing support now matter as much as torque. |

## 5. Mechanical structure outputs

| Part | Output |
|---|---|
| L1 upper link | `40 x 30 x 2.5 mm` 6061-T6 aluminium rectangular tube, tall side vertical. |
| L2 forearm link | `35 x 25 x 2.0 mm` 6061-T6 aluminium rectangular tube, tall side vertical. |
| L1 tube-only mass | `0.246 kg`; design mass retained as `0.300 kg` with attachments. |
| L2 tube-only mass | `0.157 kg`; design mass retained as `0.280 kg` with attachments. |
| L1 estimated deflection | `0.047 mm` under simplified worst tip force. |
| L2 estimated deflection | `0.035 mm` under simplified worst tip force. |
| Main structural caution | Joint plates, bearings, gearbox shafts and fasteners need CAD/detail validation. |

## 6. Accuracy and gearbox targets

| Topic / item | Output |
|---|---|
| TCP accuracy target | `+/-5 mm` |
| TCP repeatability target | `+/-3 mm` |
| Preferred loaded gearbox backlash | `<=0.15 deg` output for J2/J3 |
| Absolute maximum backlash target | `<=0.30 deg` output |
| J2 ideal microstep output | `0.00225 deg` per microstep at `1600 microsteps/rev` and `100:1` |
| Main accuracy driver | Backlash, compliance and calibration, not step resolution. |

## 7. Mechanical gripper output

| Topic / item | Output | Status |
|---|---|---|
| Type | Near-parallel two-finger mechanical gripper. | Selected |
| Actuator | Third ST3215 servo. | Selected |
| Opening target | `60-80 mm` clear opening. | Provisional |
| Finger length target | `45-60 mm`. | Provisional |
| Conservative pinch force target | About `20 N` continuous-style design estimate. | Provisional |
| Short-duration maximum force estimate | About `70 N`, not for continuous holding. | Reference only |
| Finger pads | Replaceable rubber/TPU pads. | Recommended |
| Protection | Mechanical end stops and software current/position limits. | Required |

## 8. Modular interface output

| Topic / item | Output | Status |
|---|---|---|
| Interface style | Flat face with two locating features and positive mechanical lock. | Selected |
| Pogo connector role | Electrical only; no structural load. | Selected |
| Minimum pogo contacts | 4: `+12 V`, `GND`, serial data, tool-ID. | Compatible |
| Preferred pogo contacts | 6: duplicate `+12 V` and `GND`, plus serial data and tool-ID. | Recommended |
| Pneumatic compatibility | Add lightweight pneumatic quick connector for future soft/vacuum tools. | Selected |
| Tool-change sequence | Power off, attach/lock, detect tool, enable power. | Selected |

## 9. Design freeze blockers

| Blocker | Why it matters |
|---|---|
| Select actual J1-J3 gearboxes | Need real ratio, efficiency, backlash, output bearing rating and mass. |
| Validate J2 gearbox at 500 rpm motor speed | Torque curve looks acceptable, but selected driver/current/gearbox must be tested. |
| Validate J4 continuous torque | ST3215 stall torque comparison is not enough for thermal reliability. |
| CAD collision check | Current maps are ideal kinematic maps only. |
| Replace provisional masses | Use CAD mass properties after first assembly model. |
| Pogo pin current rating | ST3215 locked current can exceed what small contacts may safely carry. |

## 10. Final file map

| File | Purpose |
|---|---|
| `README.md` | Reader-friendly entry point for the folder. |
| `Workspace_constraints` | Essential requirements and final decision pointers. |
| `Kinematic_calculations.md` | DH, forward kinematics, IK and workspace map details. |
| `Design_log.md` | Full decision and workflow log. |
| `Final_outputs.md` | Consolidated final provisional outputs. |
| `analysis_artifacts/Torque_calculations.md` | Mass and torque calculation details. |
| `analysis_artifacts/J2_gearbox_sizing_check.md` | J2 gearbox screening. |
| `analysis_artifacts/Final_design_sizing.md` | Final drivetrain, structure, gripper and interface sizing. |

## 11. Next physical work

| Step | Action |
|---|---|
| 1 | Search/select actual gearboxes matching J1 `10:1`, J2 `100:1`, J3 `40:1`. |
| 2 | Start CAD skeleton with axes, link tubes, gearbox envelopes and tool interface. |
| 3 | Check CP2 folded pose, CP1 downward pose and R600 pose in CAD for physical collisions. |
| 4 | Replace provisional masses with CAD mass properties and rerun torque calculations. |
| 5 | Prototype or CAD the mechanical gripper linkage around the ST3215. |
