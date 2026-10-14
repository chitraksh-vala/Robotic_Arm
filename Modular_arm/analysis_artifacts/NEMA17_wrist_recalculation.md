# All-Stepper Wrist Recalculation — Current Design Instance

> Superseded for the current design instance by [`Selected_products_recalculation.md`](Selected_products_recalculation.md) (2026-10-14). Retained as historical work.

## 1. Motor and wrist mass model

| Joint / item | Model | Mass used | Location used in calculation | Status |
|---|---|---:|---|---|
| J1 | 42AM08 NEMA 17 + gearbox | J1 motor/gearbox is base-mounted | Base | Retained |
| J2 | 23HS45-3504S NEMA 23 + gearbox | `1.600 kg` motor | J2 axis; included only in J1 yaw inertia at `56.5 mm` radius | Selected |
| J3 | 42AM08 NEMA 17 + gearbox | `1.160 kg` complete moving assembly | Elbow | Retained |
| J4 | 42AM08 NEMA 17 + gearbox | `0.910 kg` = `0.510 kg` motor + `0.250 kg` gearbox + `0.150 kg` support | Wrist centre | Provisional |
| J5 | 42AM08 NEMA 17 + gearbox | `0.760 kg` = `0.510 kg` motor + `0.150 kg` gearbox + `0.100 kg` support | `50 mm` distal of wrist centre | Provisional |
| G1 | ST3215 serial servo | Included in `0.500 kg` tool/interface budget | Tool | Retained |
| External encoders | None | `0 kg` | — | Confirmed |

The new local-wrist NEMA 17 assemblies increase the J2-carried mass from `3.270 kg` to **`4.640 kg`**. This is a deliberately conservative first packaging model. Moving either J4 or J5 actuator closer to J3 with a belt/shaft transmission can reduce upstream torque, but must be analysed with an actual layout.

## 2. Geometry retained

| Parameter | Value |
|---|---:|
| L1 | `280 mm` |
| L2 | `260 mm` |
| Tool offset | `100 mm` |
| Link material | `40 mm OD x 36 mm ID` 3K roll-wrapped CF tube |

## 3. Recalculated torque requirements

The required output capacity includes gravity, `60 deg/s^2` acceleration for J1-J4, 10% gravity-friction allowance where applicable, and the existing 67% design-utilization criterion.

| Joint | Target speed | Static torque | Acceleration allowance | Friction allowance | Required output capacity |
|---|---:|---:|---:|---:|---:|
| J1 yaw | `30 deg/s` | `0.000 N.m` | `1.216 N.m` | `0.200 N.m` | **`2.114 N.m`** |
| J2 shoulder pitch | `30 deg/s` | `21.506 N.m` | `1.211 N.m` | `2.151 N.m` | **`37.116 N.m`** |
| J3 elbow pitch | `30 deg/s` | `9.152 N.m` | `0.299 N.m` | `0.915 N.m` | **`15.472 N.m`** |
| J4 wrist pitch | `60 deg/s` | `1.354 N.m` | `0.011 N.m` | `0.135 N.m` | **`2.239 N.m`** |
| J5 wrist roll | `60 deg/s` | `0.307 N.m` | `0.001 N.m` | `0.031 N.m` | **`0.505 N.m`** |

## 4. Gearbox ranges

Screening uses 75% gearbox efficiency. J1/J3/J4/J5 use the previously supplied 42AM08 torque-speed curve. J2 uses a conservative `1.50 N.m` running-torque screen, equal to 50% of the NEMA 23 holding torque, until a curve for the final driver and supply voltage is confirmed.

| Joint | Motor | Gearbox range | Motor speed in range | Output margin | Minimum continuous gearbox rating | Preferred continuous rating |
|---|---|---:|---:|---:|---:|---:|
| J1 | 42AM08 NEMA 17 | **10:1–15:1** | `50–75 rpm` | **2.8–4.1x** | `2.1 N.m` | `3.2 N.m` |
| J2 | 23HS45-3504S NEMA 23 | **50:1–60:1** | `250–300 rpm` | **1.5–1.8x** | `37.1 N.m` | `55.7 N.m` |
| J3 | 42AM08 NEMA 17 | **50:1–60:1** | `250–300 rpm` | **1.6–1.9x** | `15.5 N.m` | `23.2 N.m` |
| J4 | 42AM08 NEMA 17 | **10:1–15:1** | `100–150 rpm` | **2.5–3.7x** | `2.2 N.m` | `3.4 N.m` |
| J5 | 42AM08 NEMA 17 | **5:1–10:1** | `50–100 rpm` | **5.8–11.3x** | `0.5 N.m` | `0.8 N.m` |

The required output capacity already includes a 1.49x capacity margin through the 67% utilization rule. The preferred gearbox rating adds a further 1.5x product-selection allowance.

## 5. Key design consequence

Replacing the wrist servos with steppers is feasible, but it raises the upstream demands materially:

| Joint | Prior servo-wrist capacity | New local-stepper-wrist capacity | Change |
|---|---:|---:|---:|
| J1 | `1.422 N.m` | `2.114 N.m` | +49% |
| J2 | `23.897 N.m` | `37.116 N.m` | +55% |
| J3 | `8.945 N.m` | `15.472 N.m` | +73% |

Use a structurally supported J2 output bearing and a J2 gearbox with low backlash, continuous rating of at least `37.1 N.m` (preferably `55.7 N.m`), and adequate radial/axial load rating. The original NEMA 17 driver is suitable only if its current/supply ratings match the 42AM08. J2 needs its own driver rated for at least `3.5 A/phase`.

## 6. Required CAD follow-up

1. Create the wrist layout around the actual gearbox envelopes; both NEMA 17s at the wrist may violate collision/clearance targets.
2. Replace the provisional gearbox/support masses with actual part masses and rerun this script.
3. Evaluate an elbow-mounted J4 motor with belt transmission before purchasing a 50–60:1 J2 gearbox; it is the clearest way to reduce the added J2/J3 load.
4. Confirm the exact NEMA 23 SKU includes the internal encoder, if that is required by the selected controller.

Calculation source: `nema17_wrist_recalculation.ps1`  
Machine-readable output: `nema17_wrist_recalculation_summary.json`
