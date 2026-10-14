# Final Outputs - Provisional Custom Modular Robotic Arm

Status: current provisional CF-link design output, ready for CAD concept modelling and gearbox procurement comparison. This is not a manufacturing release.

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
| J1 | Rtelligent 42AM08 + gearbox, no external encoder | `10-15:1` | `2.17 N.m` | `30 deg/s` | 2.39-3.54x margin using conservative 0.71 N.m basis. | Current provisional |
| J2 | 23HS45-3504S NEMA 23 + gearbox, no external encoder | `50-60:1` | `38.88 N.m` | `30 deg/s` | Conservative 1.45-1.74x margin; requires >=3.5 A/phase driver. | Current provisional |
| J3 | Rtelligent 42AM08 + gearbox, no external encoder | `50-60:1` | `15.88 N.m` | `30 deg/s` | 1.41-1.64x margin using conservative 0.71 N.m basis. | Current provisional |
| J4 | Rtelligent 42AM08 + gearbox, no external encoder | `10-15:1` | `2.24 N.m` | `60 deg/s` | 2.26-3.26x margin using conservative 0.71 N.m basis. | Current provisional |
| J5 | Rtelligent 42AM08 + gearbox, no external encoder | `5-10:1` | `0.51 N.m` | `60 deg/s` | 5.14-10.02x margin using conservative 0.71 N.m basis. | Current provisional |
| G1 | ST3215 serial servo | Gripper linkage | TBD | TBD | Use for mechanical gripper. | Selected |

## 4. J2 gearbox conclusion

| Topic / item | Output |
|---|---|
| Selected current J2 approach | Gearbox only, no counterbalance. |
| Selected ratio range | `50:1 to 60:1` |
| Joint speed target | `30 deg/s = 5 rpm` |
| Motor speed needed | `250-300 rpm` |
| Conservative motor torque used | `1.50 N.m`, 50 percent of the 3.0 N.m holding torque |
| Estimated available output torque | `56.25-67.50 N.m` |
| Torque margin | `1.45-1.74x` |
| J2 gearbox continuous-rating screen | at least `38.9 N.m`; prefer `58.3 N.m` |
| Main warning | Gearbox backlash and output bearing support now matter as much as torque. |

## 5. Mechanical structure outputs

| Part | Output |
|---|---|
| L1 upper link | Vortex-RC `40 mm OD x 36 mm ID` 3K roll-wrapped carbon-fibre tube. |
| L2 forearm link | Vortex-RC `40 mm OD x 36 mm ID` 3K roll-wrapped carbon-fibre tube. |
| Tube mass basis | Supplier listing `1.100 kg/m`, conservatively treated as tube mass pending weigh-in. |
| L1 tube-only / design mass | `0.308 kg` / `0.410 kg`, including bonded-insert/local-protection reserve. |
| L2 tube-only / design mass | `0.286 kg` / `0.390 kg`, including bonded-insert/local-protection reserve. |
| L1 estimated deflection / nominal stress | `0.077 mm` / `3.79 MPa` using 64 GPa conservative CF modulus. |
| L2 estimated deflection / nominal stress | `0.032 mm` / `1.83 MPa` using 64 GPa conservative CF modulus. |
| Main structural caution | Joint plates, bearings, gearbox shafts and fasteners need CAD/detail validation. |

## 6. Accuracy and gearbox targets

| Topic / item | Output |
|---|---|
| TCP accuracy target | `+/-5 mm` |
| TCP repeatability target | `+/-3 mm` |
| Preferred loaded gearbox backlash | `<=0.15 deg` output for J2/J3 |
| Absolute maximum backlash target | `<=0.30 deg` output |
| J2 ideal microstep output | `0.00450-0.00375 deg` per microstep at `1600 microsteps/rev` and `50-60:1` |
| Main accuracy driver | Backlash, compliance, homing error and possible missed steps, not nominal microstep resolution. |

## 7. Mechanical gripper output

| Topic / item | Output | Status |
|---|---|---|
| Type | Near-parallel two-finger mechanical gripper. | Selected |
| Actuator | Third ST3215 servo; retained for G1 only. | Selected |
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
| Validate J2 gearbox at 250-300 rpm motor speed | The selected driver/current/gearbox torque curve must be tested. |
| Validate wrist stepper packaging | Replace provisional J4/J5 gearbox/support masses with actual CAD mass properties and rerun the torque model. |
| CAD collision check | Current maps are ideal kinematic maps only. |
| Replace provisional masses | Use CAD mass properties after first assembly model. |
| Open-loop position validation | Without external encoders, add repeatable homing references and verify that J1-J3 do not lose steps over the required duty cycle. |
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
| `analysis_artifacts/Selected_products_recalculation.md` | Current authoritative linked-product recalculation. |

## 11. Next physical work

| Step | Action |
|---|---|
| 1 | Search/select actual gearboxes matching J1 `10-15:1`, J2 `50-60:1`, J3 `50-60:1`. |
| 2 | Start CAD skeleton with axes, link tubes, gearbox envelopes and tool interface. |
| 3 | Check CP2 folded pose, CP1 downward pose and R600 pose in CAD for physical collisions. |
| 4 | Replace provisional masses with CAD mass properties and rerun torque calculations. |
| 5 | Prototype or CAD the mechanical gripper linkage around the ST3215. |
