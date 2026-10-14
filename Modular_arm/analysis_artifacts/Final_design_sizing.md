# Final Design Sizing Calculations

> Superseded for the selected products by [`Selected_products_recalculation.md`](Selected_products_recalculation.md) (2026-10-14). Retained as historical work.

Status: superseded for current design by [`Carbon_fiber_40x36_recalculation.md`](Carbon_fiber_40x36_recalculation.md). This record preserves the earlier aluminium-link structural pass.

## 1. Final baseline selected

| Topic / item | Value | Status |
|---|---:|---|
| Shoulder height, H | `180 mm` | Selected |
| Upper-arm link, L1 | `280 mm` | Selected provisional |
| Forearm link, L2 | `260 mm` | Selected provisional |
| Tool offset, Lt | `100 mm` | Selected |
| Payload object | `0.750 kg` | Confirmed |
| Gripper/interface mass | `0.500 kg` | Provisional |
| Distal moving mass carried by J2 | `3.440 kg` | Preliminary |
| J2 counterbalance | `0 N.m` | Not used in current selected version |

## 2. Selected drivetrain check

The stepper checks use the approximate `42AM08` torque-speed curve supplied by the user:

| Motor speed | Approx available torque |
|---:|---:|
| `50 rpm` | `0.78 N.m` |
| `100 rpm` | `0.76 N.m` |
| `200 rpm` | `0.70 N.m` |
| `300 rpm` | `0.65 N.m` |
| `400 rpm` | `0.58 N.m` |
| `500 rpm` | `0.53 N.m` |
| `600 rpm` | `0.49 N.m` |

| Joint | Ratio | Required output torque | Target speed | Motor speed | Required motor torque | Curve available motor torque | Estimated available output torque | Margin | Status |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| J1 | `10:1` | `1.440 N.m` | `30 deg/s` | `50 rpm` | `0.192 N.m` | `0.780 N.m` | `5.850 N.m` | `4.06x` | Pass |
| J2 | `100:1` | `24.653 N.m` | `30 deg/s` | `500 rpm` | `0.329 N.m` | `0.530 N.m` | `39.750 N.m` | `1.61x` | Pass |
| J3 | `40:1` | `9.115 N.m` | `30 deg/s` | `200 rpm` | `0.304 N.m` | `0.700 N.m` | `21.000 N.m` | `2.30x` | Pass |

Formula:

```text
motor rpm = joint rpm * gearbox ratio
required motor torque = required output torque / (ratio * gearbox efficiency)
available output torque = curve motor torque * ratio * gearbox efficiency
```

Assumed gearbox efficiency: `75 percent`.

## 3. Output speed comparison

| Joint | Selected ratio | Motor rpm needed for target speed | Output speed | Comment |
|---|---:|---:|---:|---|
| J1 | `10:1` | `50 rpm` | `30 deg/s` | Easy for the motor. |
| J2 | `100:1` | `500 rpm` | `30 deg/s` | Workable, but most demanding. |
| J3 | `40:1` | `200 rpm` | `30 deg/s` | Comfortable. |
| J4 | Internal servo gearing | Not calculated | `60 deg/s` target | Validate on hardware. |
| J5 | Internal servo gearing | Not calculated | `60 deg/s` target | Validate on hardware. |

## 4. Servo torque check

The ST3215 stall/locked-rotor torque is treated as `30 kg.cm = 2.942 N.m`. This is not continuous torque.

| Joint | Required output torque | Assumed usable fraction of stall | Assumed usable torque | Margin | Status |
|---|---:|---:|---:|---:|---|
| J4 | `1.638 N.m` | `30 percent` | `0.883 N.m` | `0.54x` | Fails under conservative continuous assumption |
| J4 | `1.638 N.m` | `50 percent` | `1.471 N.m` | `0.90x` | Slightly low |
| J4 | `1.638 N.m` | `67 percent` | `1.971 N.m` | `1.20x` | Passes only if thermal duty is acceptable |
| J5 | `0.506 N.m` | `30 percent` | `0.883 N.m` | `1.74x` | Pass |
| J5 | `0.506 N.m` | `50 percent` | `1.471 N.m` | `2.91x` | Pass |
| J5 | `0.506 N.m` | `67 percent` | `1.971 N.m` | `3.90x` | Pass |

Decision: J5 is acceptable. J4 is acceptable only if the wrist pitch is not held for long periods near worst-case load, or if real thermal testing confirms it. CAD should try to reduce J4 tool moment.

## 5. Link structural check

Material assumption: 6061-T6 aluminium, `E = 69 GPa`, density `2700 kg/m^3`.

| Link | Selected section | Length | Tube-only mass | Design mass | Bending inertia | Tip force used | Estimated deflection | Estimated stress | Status |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| L1 upper arm | `40x30x2.5 mm` rectangular tube, tall side vertical | `280 mm` | `0.246 kg` | `0.300 kg` | `70677 mm^4` | `30.1 N` | `0.045 mm` | `2.38 MPa` | Pass |
| L2 forearm | `35x25x2.0 mm` rectangular tube, tall side vertical | `260 mm` | `0.157 kg` | `0.280 kg` | `37162 mm^4` | `15.2 N` | `0.035 mm` | `1.86 MPa` | Pass |

Formula:

```text
I = (b*h^3 - b_i*h_i^3) / 12
delta = F*L^3 / (3*E*I)
stress = M*c/I
```

These values are low because the straight tubes are not the weak point. The real mechanical risk is joint bracket stiffness, gearbox shaft/bearing support, fastener preload and printed/adaptor parts.

## 6. Accuracy and backlash targets

| Topic / item | Calculation | Requirement generated |
|---|---|---|
| TCP accuracy | `5 mm / 640 mm = 0.0078125 rad = 0.447 deg` | Keep total loaded angular error under about `0.45 deg`. |
| TCP repeatability | `3 mm / 640 mm = 0.0046875 rad = 0.269 deg` | Keep repeatable loaded angular error under about `0.27 deg`. |
| Gearbox backlash | J2 and J3 errors both move the TCP significantly. | Prefer `<=0.15 deg` output backlash per loaded gearbox, absolute maximum `<=0.30 deg`. |
| Microstep output | J2: `360/(1600*100) = 0.00225 deg`. | Resolution is far smaller than the accuracy target. Backlash, open-loop step loss and compliance dominate. |

## 7. Mechanical gripper sizing

| Topic / item | Value | Calculation | Status |
|---|---:|---|---|
| Gripper actuator | `1x ST3215` | Third Waveshare servo. | Confirmed |
| Stall torque reference | `2.942 N.m` | `30 kg.cm * 0.0980665`. | Reference |
| Conservative usable torque | `0.883 N.m` | `30 percent` of stall torque. | Provisional |
| Effective finger radius | `25 mm` | First linkage sizing value. | Provisional |
| Linkage efficiency | `60 percent` | Conservative first value. | Provisional |
| Conservative pinch force | `21.2 N` | `0.883 * 0.60 / 0.025`. | Preliminary |
| Stall-limited pinch force | `70.6 N` | `2.942 * 0.60 / 0.025`. | Do not use continuously |

Design direction: make a near-parallel two-finger gripper with `60-80 mm` opening, replaceable soft pads, mechanical end stops, and no requirement to hold stall torque continuously.

## 8. Modular interface sizing

| Topic / item | Design output | Status |
|---|---|---|
| Mechanical alignment | Two dowels/locating pins plus a flat face. | Selected |
| Mechanical locking | Positive latch, screw clamp, wedge or over-centre latch. | Selected |
| Electrical contacts | Minimum 4: `+12 V`, `GND`, serial data, tool-ID. | Compatible |
| Preferred electrical contacts | 6: duplicate `+12 V` and `GND`, plus serial data and tool-ID. | Recommended |
| Pogo pin role | Electrical only; no structural load. | Confirmed |
| Soft gripper pneumatic path | Pump/regulator off-arm, lightweight tube and quick connector on tool side. | Selected |

## 9. Final design checks

| Check | Result | Status |
|---|---|---|
| Required critical poses | Pass in existing workspace analysis. | Complete |
| CP2-to-CP1 downward continuity | Pass with negative-elbow branch. | Complete |
| J2 gearbox-only feasibility | Pass concept with `100:1` and supplied motor curve. | Complete |
| J3 gearbox feasibility | Pass concept with `40:1`. | Complete |
| Link tube bending/deflection | Pass concept. | Complete |
| J4 servo continuous torque | Not fully proven. | Validation required |
| Physical collision/clearance | Not checked without CAD model. | Required |
| Gearbox backlash/output bearing | Not checked without selected gearbox. | Required |

Calculation source file: `final_design_sizing.ps1`  
Machine-readable output: `final_design_sizing_summary.json`
