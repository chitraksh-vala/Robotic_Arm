# Carbon-Fibre 40 mm OD / 36 mm ID Recalculation

> Superseded for the current design instance by [`Selected_products_recalculation.md`](Selected_products_recalculation.md) (2026-10-14). Retained as historical work.

## 1. Design instance and assumptions

| Topic / item | Value used | Status |
|---|---:|---|
| Link material | 3K roll-wrapped carbon-fibre twill, matte finish | User specified |
| Link section, both L1 and L2 | `40 mm OD x 36 mm ID` round tube; `2 mm` wall | User specified |
| CF density used | `1.60 g/cm^3` | Conservative provisional |
| Longitudinal bending modulus used | `64 GPa` | Conservative provisional; replace with supplier laminate datasheet |
| L1 bare tube mass | `0.107 kg` | Calculated |
| L2 bare tube mass | `0.099 kg` | Calculated |
| L1 design mass in torque model | `0.210 kg` | Includes `0.103 kg` reserve for bonded end inserts/local protection |
| L2 design mass in torque model | `0.200 kg` | Includes `0.101 kg` reserve for bonded end inserts/local protection |
| External encoder mass | `0 kg` | User directed |
| NEMA internal encoder | Included within the `0.510 kg` stated NEMA motor mass; not added separately | User directed |
| J3 moving assembly | `1.160 kg` = motor with internal encoder + assumed gearbox + brackets/bearings | Provisional |
| Payload / tool mass | `0.750 kg` / `0.500 kg` | Payload confirmed; tool mass provisional |

## 2. Selected link lengths

| Link | Selected length | Result |
|---|---:|---|
| L1, shoulder to elbow | **280 mm** | Retained; all required critical poses pass |
| L2, elbow to wrist | **260 mm** | Retained; all required critical poses pass |
| Tool offset, wrist to TCP | `100 mm` | Unchanged |

The link-length sweep was rerun with the CF link masses. Moving to `250/290 mm` reduces J2 capacity only from `23.897 N.m` to `23.312 N.m` (about `2.4%`) but raises J3 from `8.945 N.m` to `9.887 N.m`. Therefore `280/260 mm` remains the selected balanced geometry.

## 3. Current torque calculation

Worst case is the arm extended horizontally. The demand includes gravity, a `60 deg/s^2` acceleration allowance for J1-J3, a 10% gravity-friction allowance for pitch joints, and a 67% continuous-output utilization target.

| Joint | Static gravity | Acceleration allowance | Friction allowance | Preliminary demand | Required minimum output capacity |
|---|---:|---:|---:|---:|---:|
| J1 base yaw | `0.000 N.m` | `0.748 N.m` | `0.200 N.m` | `0.948 N.m` | **`1.414 N.m`** |
| J2 shoulder pitch | `13.876 N.m` | `0.748 N.m` | `1.388 N.m` | `16.011 N.m` | **`23.897 N.m`** |
| J3 elbow pitch | `5.285 N.m` | `0.179 N.m` | `0.529 N.m` | `5.993 N.m` | **`8.945 N.m`** |

The CF links reduce the J2-carried moving mass to `3.270 kg`; the previous no-external-encoder aluminium-link model was `3.440 kg`.

## 4. Gearbox selection ranges and margins

The margins below use the user-supplied 42AM08 torque-speed curve and an assumed 75% gearbox efficiency. They are concept-stage margins, not gearbox purchase approval. Select a gearbox whose **continuous output torque rating** is at least `1.5x` the required output torque, preferably `2x` for J2, and confirm its backlash and output-bearing load rating.

| Joint | Gearbox range to select | Curve-based margin in recommended range | Selection direction |
|---|---|---:|---|
| J1 | **10:1 to 15:1** planetary gearbox | At `10:1`, about **4.1x** at `30 deg/s` | Torque is comfortable; prioritize low backlash, stiffness and compact packaging. |
| J2 | **100:1 to 120:1** planetary gearbox | About **1.7x to 1.8x** at `30 deg/s` | Critical joint. Prefer the lower-backlash, better-bearing-supported option; do not choose below `100:1` without a verified torque-speed curve. |
| J3 | **35:1 to 50:1** planetary gearbox | About **2.1x to 2.8x** at `30 deg/s` | `40:1` is a good centre of the range; select based on backlash, mass and packaging. |

The calculated motor/output check at the centre selections is:

| Joint | Check ratio | Motor speed at 30 deg/s joint speed | Required motor torque | Curve-estimated output torque | Margin |
|---|---:|---:|---:|---:|---:|
| J1 | `10:1` | `50 rpm` | `0.189 N.m` | `5.85 N.m` | **4.14x** |
| J2 | `100:1` | `500 rpm` | `0.319 N.m` | `39.75 N.m` | **1.66x** |
| J3 | `40:1` | `200 rpm` | `0.298 N.m` | `21.00 N.m` | **2.35x** |

## 5. Carbon-tube structural screen

| Link | Bending inertia | Simplified tip load | End deflection | Nominal bending stress | Result |
|---|---:|---:|---:|---:|---|
| L1 | `43,216 mm^4` | `29.3 N` | `0.077 mm` | `3.79 MPa` | Pass at concept stage |
| L2 | `43,216 mm^4` | `15.2 N` | `0.032 mm` | `1.83 MPa` | Pass at concept stage |

These are tube-body values only. The governing details are likely to be bonded aluminium inserts, clamp/contact pressure, bracket stiffness, gearbox-output bearing support, and any holes or cut-outs. Do not drill bare structural holes through the CF tube; use bonded inserts or external clamps, electrically isolated from the CF where required to prevent galvanic corrosion.

## 6. Required validation before purchase/fabrication

1. Obtain the tube supplier's longitudinal flexural modulus, fibre layup, compressive strength and recommended insert-bonding method. “3K twill” identifies fibre tow/surface, not the complete structural laminate.
2. Confirm J2 gearbox continuous torque, backlash, output radial/axial load rating, efficiency at approximately `500 rpm` motor speed, and mass.
3. Use homing/limit switches and perform missed-step testing. The internal NEMA encoder is not assigned any extra mass, but its feedback capability must be verified with the selected drive/controller.
4. Replace provisional insert, bracket, gearbox and gripper masses with CAD mass properties, then rerun the scripts.

Calculation source files: `torque_model.ps1`, `link_length_torque_sweep.ps1`, `j2_torque_reduction_study.ps1`, `j2_gearbox_sizing_check.ps1`, and `final_design_sizing.ps1`.
