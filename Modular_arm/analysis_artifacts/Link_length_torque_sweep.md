# Link-Length Torque Redistribution Sweep

Status: preliminary calculation. This checks whether changing the L1/L2 split can reduce the high J2 shoulder torque without changing total reach.

## 1. Sweep setup

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Total reach | Keep `L1 + L2 + Lt = 640 mm` for all candidates. | Controlled |
| Tool offset | Keep `Lt = 100 mm`. | Confirmed |
| Shoulder height | Keep `H = 180 mm`. | Confirmed |
| Payload | Keep object payload at `0.75 kg`. | Confirmed |
| Tool mass | Keep gripper/interface mass at `0.50 kg`. | Provisional |
| J3 assembly | Keep moving J3 motor/encoder/gearbox/bracket assembly at `1.26 kg`. | Provisional |
| Wrist assembly | Keep wrist assembly at `0.30 kg`. | Provisional |
| Link masses | Scale structural link and cable masses with link length. | Provisional |
| Critical poses | Check CP1, CP2, CP3 and R600 for every candidate. | Done |

## 2. Candidate comparison

| Candidate | L1 (mm) | L2 (mm) | Distal moving mass (kg) | J2 min output capacity (N.m) | J3 min output capacity (N.m) | J2/J3 torque ratio | Critical poses pass? |
|---|---:|---:|---:|---:|---:|---:|---|
| Current | 280 | 260 | 3.540 | 25.117 | 9.115 | 2.76 | Yes |
| A | 260 | 280 | 3.541 | 24.692 | 9.769 | 2.53 | Yes |
| B | 250 | 290 | 3.541 | 24.480 | 10.100 | 2.42 | Yes |
| C | 240 | 300 | 3.541 | 24.268 | 10.432 | 2.33 | Yes |
| D | 270 | 270 | 3.540 | 24.904 | 9.441 | 2.64 | Yes |

## 3. Change relative to current baseline

| Candidate | J2 change | J3 change | Interpretation | Status |
|---|---:|---:|---|---|
| A, 260/280 | -1.7 percent | +7.2 percent | Mild redistribution with low disruption. | Viable |
| B, 250/290 | -2.5 percent | +10.8 percent | Better balance, still conservative. | Recommended trial |
| C, 240/300 | -3.4 percent | +14.5 percent | Best torque balance from this sweep, but higher elbow load. | Aggressive trial |
| D, 270/270 | -0.8 percent | +3.6 percent | Very small benefit. | Low value |

## 4. CP2 folded-pose check

CP2 is the low inward pose: R = 200 mm, Z = 0 mm, downward tool. It is the pose most likely to become difficult when L1 is shortened.

| Candidate | Valid CP2 branch | J2 angle (deg) | J3 angle (deg) | J4 angle (deg) | Status |
|---|---|---:|---:|---:|---|
| Current | Negative elbow | 39.89 | -133.16 | 3.27 | Pass |
| A | Negative elbow | 49.67 | -133.16 | -6.50 | Pass |
| B | Negative elbow | 54.89 | -133.71 | -11.17 | Pass |
| C | Negative elbow | 60.42 | -134.65 | -15.77 | Pass |
| D | Negative elbow | 44.69 | -132.98 | -1.71 | Pass |

## 5. Conclusion

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Can link split redistribute torque? | Yes. Shortening L1 and lengthening L2 reduces J2 torque and increases J3 torque. | Confirmed |
| Is the improvement large? | No. J2 drops only about 1.7 to 3.4 percent across the practical candidates because the total reach and distal payload/tool mass are unchanged. | Important |
| Best balanced option | `L1 = 240 mm`, `L2 = 300 mm` gives the lowest J2/J3 ratio, but increases J3 capacity requirement to `10.432 N.m`. | Aggressive |
| Recommended engineering trial | `L1 = 250 mm`, `L2 = 290 mm` is the best compromise from this sweep. It reduces J2/J3 ratio from `2.76` to `2.42` while keeping all required critical poses valid. | Recommended for next CAD trial |
| Main design lesson | Link split helps balance torque, but it will not solve the J2 torque problem by itself. For a meaningful J2 reduction, reduce distal mass, move mass closer to the shoulder, or add counterbalance. | Key conclusion |

Generated numerical summary: `link_length_torque_sweep_summary.json`  
Calculation source file: `link_length_torque_sweep.ps1`
