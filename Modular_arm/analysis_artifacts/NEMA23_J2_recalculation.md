# NEMA 23 J2 Recalculation — Current Design Instance

> Superseded for the current design instance by [`Selected_products_recalculation.md`](Selected_products_recalculation.md) (2026-10-14). Retained as historical work.

## 1. Motor allocation and assumptions

| Item | Selection / calculation treatment | Status |
|---|---|---|
| J1 | Original Rtelligent 42AM08 NEMA 17 | Retained |
| J2 | StepperOnline `23HS45-3504S` NEMA 23 | User selected |
| J3 | Original Rtelligent 42AM08 NEMA 17 | Retained |
| J2 holding torque | `3.00 N.m` | Supplied datasheet / manufacturer specification |
| J2 rated current | `3.5 A/phase` | Supplied datasheet / manufacturer specification |
| J2 mass | `1.60 kg` | Manufacturer specification |
| J2 motor geometry | `57 x 57 mm` frame, `113 mm` body, `10 mm` shaft | Supplied datasheet / manufacturer specification |
| External encoders | `0 kg`; not used | User directed |
| Internal encoders | Treated as part of the stated motor mass, not separately added | User directed |
| J2 running-torque screen | `1.50 N.m`, or 50% of holding torque | Conservative preliminary assumption pending the selected driver/supply torque-speed curve |
| Gearbox efficiency | `75%` | Preliminary conservative assumption |
| CF links | Both links use `40 mm OD x 36 mm ID` 3K roll-wrapped CF tube | Current design instance |

Important purchase check: the standard four-wire `23HS45-3504S` product listing and drawing identify a standard stepper and do not explicitly state an integrated encoder. Confirm the encoder-equipped exact SKU before procurement. This calculation still adds **no encoder mass** as requested.

## 2. Geometry and mass model

| Item | Value | Status |
|---|---:|---|
| L1 shoulder-to-elbow | `280 mm` | Selected |
| L2 elbow-to-wrist | `260 mm` | Selected |
| Tool offset | `100 mm` | Selected |
| L1 design mass | `0.210 kg` | CF tube plus bonded-insert/local-protection reserve |
| L2 design mass | `0.200 kg` | CF tube plus bonded-insert/local-protection reserve |
| J2-carried distal mass | `3.270 kg` | Excludes J2 motor and J2 gearbox because they are on the J2 axis |
| NEMA 23 effect on J1 | `1.60 kg` at a `56.5 mm` effective yaw radius | Included in J1 acceleration only |

The NEMA 23 does not increase J2 pitch gravity torque because its centre is on the J2 pitch axis. It adds a small J1 yaw-inertia contribution because its body extends away from the J1 axis.

## 3. Recalculated torque requirements

The output capacity includes gravity, `60 deg/s^2` acceleration for J1-J3, friction allowance, and the existing 67% utilization design criterion.

| Joint | Static torque | Acceleration allowance | Friction allowance | Preliminary demand | Required output capacity |
|---|---:|---:|---:|---:|---:|
| J1 yaw | `0.000 N.m` | `0.753 N.m` | `0.200 N.m` | `0.953 N.m` | **`1.422 N.m`** |
| J2 shoulder pitch | `13.876 N.m` | `0.748 N.m` | `1.388 N.m` | `16.011 N.m` | **`23.897 N.m`** |
| J3 elbow pitch | `5.285 N.m` | `0.179 N.m` | `0.529 N.m` | `5.993 N.m` | **`8.945 N.m`** |

## 4. Gearbox screening at 30 deg/s joint speed

For purchase screening, select the gearbox for **at least 1.5x** the required output capacity in continuous torque, and prefer **2x** on J2. Backlash, output bearing rating and stiffness remain mandatory checks.

| Joint | Ratio range to select | Output margin in range | Gearbox continuous torque rating | Reason |
|---|---|---:|---:|---|
| J1, NEMA 17 | **10:1 to 15:1** | **4.1x to 6.1x** | at least `2.1 N.m`; prefer `2.8 N.m` | Choose for low backlash and mechanical packaging, not torque. |
| J2, NEMA 23 | **30:1 to 40:1** | **1.4x to 1.9x** | at least `35.8 N.m`; prefer `47.8 N.m` | `35:1` is the balanced screening point. Use a real curve before choosing the lower end. |
| J3, NEMA 17 | **35:1 to 50:1** | **2.1x to 2.8x** | at least `13.4 N.m`; prefer `17.9 N.m` | `40:1` remains the balanced screening point. |

The J2 NEMA 23 screening deliberately uses only 50% of the 3.0 N.m holding torque, producing the following conservative outcomes:

| J2 ratio | Motor speed | Conservatively available output | Margin over 23.897 N.m requirement | Decision |
|---:|---:|---:|---:|---|
| `25:1` | `125 rpm` | `28.1 N.m` | `1.18x` | Too little margin |
| `30:1` | `150 rpm` | `33.8 N.m` | `1.41x` | Lower acceptable boundary after curve validation |
| `35:1` | `175 rpm` | `39.4 N.m` | `1.65x` | Preferred screening point |
| `40:1` | `200 rpm` | `45.0 N.m` | `1.88x` | Highest torque margin; check backlash/size |

The J1 and J3 ranges use the previously supplied 42AM08 torque-speed curve. The J2 range uses a conservative fraction because the manufacturer curve is tied to a particular drive/supply test condition and must not be assumed valid with an arbitrary 24 V driver.

## 5. Electrical and mechanical implications

| Item | Requirement / decision |
|---|---|
| J2 driver | Select a separate bipolar stepper driver rated for at least `3.5 A/phase` and the chosen supply voltage. Do not assume the original NEMA 17 driver can deliver this current. |
| J2 gearbox interface | Must accept the NEMA 23 `57 mm` mounting frame and `10 mm` shaft, or use a correctly sized adapter/coupler. |
| J2 structural support | Put the shoulder output on its own bearing pair or a gearbox explicitly rated for the radial/overhung arm load; do not hang the arm from an unsupported gearbox shaft. |
| CAD update | Include the `113 mm` NEMA 23 body and actual gearbox envelope before validating CP2 and self-collision. |
| Mass update | Add the selected J2 gearbox mass and centre of mass to the J1 yaw calculation after choosing a specific part. |

Calculation source: `nema23_j2_recalculation.ps1`  
Machine-readable output: `nema23_j2_recalculation_summary.json`
