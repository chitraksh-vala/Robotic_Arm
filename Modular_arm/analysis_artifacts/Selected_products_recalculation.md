# Selected-Products Recalculation — 2026-10-14

Status: current authoritative mechanical/drivetrain sizing pass. This replaces the earlier carbon-fibre and all-stepper torque summaries where their values differ.

## Product basis

| Item | Product used | Calculation value | Status |
|---|---|---:|---|
| L1 and L2 tube | Vortex-RC 40 x 36 x 1000 mm, 3K roll-wrapped CF tube | 40 mm OD, 36 mm ID; **1.100 kg/m listing mass** | Working conservative value |
| J1/J3/J4/J5 | Rtelligent 42AM08 NEMA 17 | **0.71 N.m** holding-torque basis; 0.510 kg each | Conservative: URL and manufacturer sheet conflict (0.71 versus 0.80 N.m typical) |
| J2 | StepperOnline 23HS45-3504S NEMA 23 | 3.0 N.m holding torque; 1.600 kg; 3.5 A/phase | Selected |
| G1 | ST3215 servo | Included within the 0.500 kg tool/interface allowance | Selected |

The tube listing presents `1.1 kg` in its additional-information field but does not state clearly whether this is a part or shipping mass. The calculation deliberately treats it as **1.100 kg per metre**, which is conservative. Weigh the purchased one-metre tube and update this value before hardware release.

## Geometry and moving masses

| Item | Value |
|---|---:|
| L1 / L2 / tool offset | 280 / 260 / 100 mm |
| L1 tube cut mass | 0.308 kg |
| L1 design mass including 0.102 kg insert/protection allowance | 0.410 kg |
| L2 tube cut mass | 0.286 kg |
| L2 design mass including 0.104 kg insert/protection allowance | 0.390 kg |
| J3 moving assembly | 1.160 kg |
| J4 wrist assembly | 0.910 kg |
| J5 wrist assembly | 0.760 kg |
| Tool/interface and payload | 0.500 kg and 0.750 kg |
| J2-carried distal mass | 5.030 kg |

No external encoder mass is included. The linked motor listings do not establish an integrated encoder; confirm the exact purchased SKU if closed-loop feedback is required.

## Method

Worst-case gravity is calculated with the arm horizontal. For each joint, the required capacity is:

`(gravity torque + inertia × 60 deg/s² + friction allowance) / 0.67`

Pitch-joint friction allowance is 10% of gravity torque; J1 has a 0.20 N.m allowance. Gearbox efficiency is 75%. NEMA-17 running torque uses the earlier supplied torque-speed curve scaled from 0.80 to 0.71 N.m; this is a conservative provisional curve pending a matching 24 V driver curve. J2 is screened at 1.50 N.m running torque (50% of its 3.0 N.m holding torque).

## Torque and gearbox result

| Joint | Required output capacity | Recommended ratio | Screen margin | Gearbox rating: minimum / preferred continuous |
|---|---:|---:|---:|---:|
| J1 yaw | **2.17 N.m** | 10:1–15:1 | 2.39–3.54x | 2.2 / 3.3 N.m |
| J2 shoulder pitch | **38.88 N.m** | 50:1–60:1 | 1.45–1.74x | 38.9 / 58.3 N.m |
| J3 elbow pitch | **15.88 N.m** | 50:1–60:1 | 1.41–1.64x | 15.9 / 23.8 N.m |
| J4 wrist pitch | **2.24 N.m** | 10:1–15:1 | 2.26–3.26x | 2.2 / 3.4 N.m |
| J5 wrist roll | **0.51 N.m** | 5:1–10:1 | 5.14–10.02x | 0.5 / 0.8 N.m |

At target joint speed, J2's 50:1–60:1 range corresponds to 250–300 rpm. J3 uses the same motor-rpm range. A selected gearbox must also meet backlash, output-bearing load, stiffness, life, mass, and mounting-envelope requirements; ratio and torque alone are insufficient.

## Result and limitations

The selected products are mechanically plausible for this concept baseline. J2 and J3 remain the critical purchase decisions. Do not select a 50:1 J2 or J3 gearbox without the manufacturer’s continuous rating and a motor/driver torque-speed curve. The local wrist-mounted stepper layout is conservative; moving wrist actuators proximally would reduce J2/J3 demand.

Reproducible source: `nema17_wrist_recalculation.ps1`.
