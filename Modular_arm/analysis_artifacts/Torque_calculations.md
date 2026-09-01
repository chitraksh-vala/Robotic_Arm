# Preliminary Torque Calculations

Status: preliminary CAD-independent estimate. These values are for early actuator and gearbox sizing only; they are not a design-freeze result.

## 1. Scope

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Arm model | Uses the current baseline geometry: H = 180 mm, L1 = 280 mm, L2 = 260 mm, Lt = 100 mm. | Current baseline |
| Joint layout | Uses SO-ARM101-style yaw-pitch-pitch-pitch-roll arrangement: J1 yaw, J2 shoulder pitch, J3 elbow pitch, J4 wrist pitch, J5 wrist roll, G1 gripper. | Confirmed |
| Payload | 750 g is treated as object-only load. | Confirmed |
| End-effector mass | 500 g is used for the gripper plus modular interface. | Provisional |
| Total tool-end sizing load | Payload plus end-effector assembly = 1.25 kg. | Provisional |
| Torque type | Calculates gravity torque, simple acceleration allowance, friction allowance and minimum output capacity. | Preliminary |

## 2. Source and assumption split

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Rtelligent 42AM08 mass | Use 0.51 kg per motor for moving-mass estimation. | Source-backed |
| Rtelligent 42AM08 torque | Use 0.80 N.m holding torque from the checked website/datasheet value. | Confirmed |
| ST3215 servo | Use 30 kg.cm locked-rotor torque at 12 V as a comparison value, not as guaranteed continuous torque. | Source-backed, continuous TBD |
| ST3215 mass | Use 70 g per servo where exact approved-BOM mass is not yet locked. | Provisional |
| Omron E6B2 encoder | Use about 100 g per encoder. | Source-backed approximation |
| RMCS-1120 driver | Excluded from moving-arm mass because the driver should be fixed off-arm. | Confirmed approach |
| Gripper-A complete mass | Official complete gripper mass is about 232 g, but the custom gripper/interface budget remains 500 g for conservative sizing. | Reference only |

## 3. Mass model

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Upper link | 0.300 kg, centre of mass at L1/2. | Provisional |
| Upper cable/tubing | 0.075 kg, centre of mass at L1/2. | Provisional |
| J3 assembly | 1.260 kg = motor, encoder, assumed gearbox, brackets and bearings. | Provisional |
| Forearm link | 0.280 kg, centre of mass at L2/2. | Provisional |
| Forearm cable/tubing | 0.075 kg, centre of mass at L2/2. | Provisional |
| Wrist assembly | 0.300 kg for two ST3215 servos plus brackets and bearings. | Provisional |
| Tool assembly | 0.500 kg at 50 mm from wrist centre. | Provisional |
| Payload | 0.750 kg at TCP, 100 mm from wrist centre. | Confirmed load |
| Total distal moving mass carried by J2 | 3.540 kg. | Preliminary |

## 4. Calculation method

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Gravity torque | For each joint, sum `m * g * horizontal distance` for all masses beyond that joint. | Used |
| J2 gravity torque | Includes upper link/cable, J3 assembly, forearm link/cable, wrist assembly, tool assembly and payload. | Used |
| J3 gravity torque | Includes forearm link/cable, wrist assembly, tool assembly and payload. | Used |
| J4 gravity torque | Includes tool assembly and payload only. | Used |
| J5 torque | Uses a provisional 25 mm eccentricity of the tool-end load about the roll axis. | Provisional |
| Acceleration allowance | Uses 60 deg/s^2 for J1-J3 and 120 deg/s^2 for J4-J5. | Provisional |
| Friction allowance | Adds 10 percent of static gravity torque, except J1 uses a provisional 0.20 N.m allowance. | Provisional |
| Sizing margin | Required continuous torque should be no more than 67 percent of available output torque. | Provisional |

## 5. Critical-pose gravity torque

| Pose | Branch | Joint angles J2/J3/J4 (deg) | Valid? | J2 gravity (N.m) | J3 gravity (N.m) | J4 gravity (N.m) | Status |
|---|---:|---:|---|---:|---:|---:|---|
| CP1, R500 Z0 downward | Negative elbow | 10.47 / -40.69 / -59.78 | Yes | 12.863 | 3.807 | 0.000 | Pass |
| CP1, R500 Z0 downward | Positive elbow | -28.65 / 40.69 / -102.04 | Yes | 12.390 | 4.309 | 0.000 | Pass |
| CP2, R200 Z0 downward | Negative elbow | 39.89 / -133.16 / 3.27 | Yes | 6.814 | 0.251 | 0.000 | Pass |
| CP2, R200 Z0 downward | Positive elbow | -83.50 / 133.16 / -139.67 | No | 3.895 | 2.852 | 0.000 | Fails J4 limit |
| CP3, R300 Z650 horizontal | Negative elbow | 85.17 / -37.89 / -47.27 | Yes | 4.746 | 3.971 | 0.981 | Pass |
| CP3, R300 Z650 horizontal | Positive elbow | 48.73 / 37.89 / -86.62 | Yes | 7.315 | 1.240 | 0.981 | Pass |
| R600, Z180 horizontal | Negative elbow | 21.34 / -44.42 / 23.07 | Yes | 13.612 | 5.035 | 0.981 | Pass |
| R600, Z180 horizontal | Positive elbow | -21.34 / 44.42 / -23.07 | Yes | 13.612 | 5.035 | 0.981 | Pass |

## 6. Worst simple static case

| Topic / item | Plan / execution description | Status |
|---|---:|---|
| Full-horizontal J2 gravity torque | 14.596 N.m | Preliminary worst case |
| Full-horizontal J3 gravity torque | 5.387 N.m | Preliminary worst case |
| Full-horizontal J4 gravity torque | 0.981 N.m | Preliminary worst case |

This full-horizontal case is slightly worse for J2 and J3 than the required critical poses, so it is used for the first sizing pass.

## 7. Preliminary output torque requirement

| Joint | Static gravity (N.m) | Acceleration allowance (N.m) | Friction allowance (N.m) | Preliminary demand (N.m) | Minimum output capacity (N.m) | Status |
|---|---:|---:|---:|---:|---:|---|
| J1 base yaw | 0.000 | 0.773 | 0.200 | 0.973 | 1.452 | Preliminary |
| J2 shoulder pitch | 14.596 | 0.773 | 1.460 | 16.829 | 25.117 | Preliminary, most critical |
| J3 elbow pitch | 5.387 | 0.181 | 0.539 | 6.107 | 9.115 | Preliminary |
| J4 wrist pitch | 0.981 | 0.018 | 0.098 | 1.097 | 1.638 | Preliminary |
| J5 wrist roll | 0.307 | 0.002 | 0.031 | 0.339 | 0.506 | Preliminary |

## 8. Actuator and gearbox screening

| Topic / item | Plan / execution description | Status |
|---|---|---|
| J1 gearbox | Mathematical ratio is about 4.8:1 with the current assumptions; screen at least 10:1 for margin and stiffness. | Provisional |
| J2 gearbox | Mathematical ratio is about 83.7:1; screen about 90:1 or higher before torque-speed validation. | Provisional, critical |
| J3 gearbox | Mathematical ratio is about 30.4:1; screen about 35:1 or higher before torque-speed validation. | Provisional |
| J4 ST3215 | Required output capacity is 1.638 N.m. ST3215 locked-rotor torque is about 2.94 N.m at 12 V, so stall comparison passes, but continuous thermal capacity is still unknown. | Validation required |
| J5 ST3215 | Required output capacity is 0.506 N.m with 25 mm eccentricity assumption; likely acceptable, but depends on actual tool centre of mass. | Validation required |

Stepper ratio screening assumes 0.80 N.m holding torque, only 50 percent usable running torque and 75 percent gearbox efficiency. This is intentionally conservative for a first pass, but final selection needs the actual torque-speed curve, gearbox efficiency and required joint speed.

## 9. Main conclusion

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Torque knowledge for current version | We now have a preliminary torque estimate for the current geometry and mass assumptions. | Complete for concept stage |
| Biggest risk | J2 shoulder pitch is the sizing driver at about 25.1 N.m required output capacity. | Critical |
| Next required data | Gearbox ratio, gearbox efficiency, gearbox mass, output bearing capacity and backlash for J1-J3. | Needed before mechanical design freeze |
| CAD validation | Replace link and bracket mass assumptions with actual CAD mass properties, then rerun this model. | Required |

Calculation source file: `torque_model.ps1`  
Generated numerical summary: `torque_model_summary.json`
