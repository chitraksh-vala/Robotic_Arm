# J2 Torque Reduction Study

Status: preliminary concept calculation. This study compares ways to reduce the high shoulder-pitch torque requirement at J2.

## 1. Baseline

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Baseline geometry | L1 = 280 mm, L2 = 260 mm, Lt = 100 mm. | Current provisional |
| Payload | Keep payload fixed at 0.75 kg object-only load. | Confirmed |
| Baseline J2 minimum output capacity | 24.653 N.m. | Current torque target |
| Main cause | J2 supports almost all distal mass: upper link, J3 assembly, forearm, wrist, tool assembly and payload. | Confirmed |

## 2. Reduction options compared

| Scenario | L1 (mm) | L2 (mm) | Distal moving mass (kg) | J2 min output capacity (N.m) | Reduction vs baseline | Critical poses pass? |
|---|---:|---:|---:|---:|---:|---|
| Baseline | 280 | 260 | 3.440 | 24.654 | 0.0 percent | Yes |
| Link split B | 250 | 290 | 3.441 | 24.068 | 2.4 percent | Yes |
| Light wrist | 280 | 260 | 3.360 | 23.921 | 3.0 percent | Yes |
| Light J3 assembly | 280 | 260 | 3.280 | 23.912 | 3.0 percent | Yes |
| Light tool | 280 | 260 | 3.290 | 23.147 | 6.1 percent | Yes |
| Very light tool | 280 | 260 | 3.190 | 22.142 | 10.2 percent | Yes |
| Mass optimization combo | 250 | 290 | 3.051 | 21.169 | 14.1 percent | Yes |
| Counterbalance 5 N.m | 280 | 260 | 3.440 | 16.445 | 33.3 percent | Yes |
| Counterbalance 8 N.m | 280 | 260 | 3.440 | 11.519 | 53.3 percent | Yes |
| Mass combo + 5 N.m counterbalance | 250 | 290 | 3.051 | 12.960 | 47.4 percent | Yes |

## 3. Interpretation

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Link split alone | Changing from L1/L2 = 280/260 to 250/290 reduces J2 capacity only from 24.653 N.m to 24.068 N.m. | Small improvement |
| Distal mass reduction | Reducing the tool and J3 assembly helps more than link split because those masses create a long shoulder moment arm. | Useful |
| Counterbalance | A 5 N.m shoulder counterbalance reduces the J2 capacity target to 16.445 N.m. An 8 N.m counterbalance reduces it to 11.519 N.m. | Strongest lever |
| Best practical direction | Combine mass reduction with a moderate counterbalance. This can bring J2 from about 24.7 N.m down near 13 N.m without changing the required poses. | Recommended |

## 4. Recommendation

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Do not rely on link lengths only | Link split improves torque balance, but it does not solve the J2 torque problem. | Recommended |
| Add J2 counterbalance | Design a shoulder counterbalance in the 5-8 N.m range near the horizontal/high-torque region. | Recommended |
| Reduce distal mass | Target tool/interface mass <= 350 g if possible, wrist assembly <= 220 g if possible, and J3 gearbox/bracket mass reduction. | Recommended |
| Keep payload unchanged | Do not reduce the 750 g object payload requirement unless the project requirement changes. | Confirmed |
| Next validation | Replace the constant counterbalance assumption with a torque-vs-angle spring/gas-strut model. | Next calculation |

## 5. Important limitation

The counterbalance values here are treated as equivalent near-horizontal shoulder torque. A real spring, gas strut, constant-force spring, elastic element or pulley counterweight will not produce perfectly constant torque across the full J2 range. The next design step must size the counterbalance geometry and check torque versus J2 angle.

Generated numerical summary: `j2_torque_reduction_study_summary.json`  
Calculation source file: `j2_torque_reduction_study.ps1`
