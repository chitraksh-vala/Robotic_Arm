# J2 Gearbox Sizing Check

Status: preliminary screening calculation. This is not a final gearbox selection because the actual stepper torque-speed curve, gearbox efficiency, backlash, output bearing rating and gearbox mass are still required.

## 1. Inputs

| Topic / item | Plan / execution description | Status |
|---|---|---|
| J2 current required output capacity | 25.117 N.m from the preliminary torque model. | Current baseline |
| Stepper motor | Rtelligent 42AM08 using checked holding torque value of 0.80 N.m. | Confirmed for screening |
| Target J2 joint speed | 30 deg/s = 5 rpm at the joint output. | Provisional |
| Gearbox efficiencies checked | 65 percent, 75 percent and 85 percent. | Screening range |
| Main table efficiency | 75 percent is used for the decision table below. | Provisional |
| Formula | Required motor torque = required output capacity / (gearbox ratio * gearbox efficiency). | Used |
| Motor speed | Motor rpm = joint rpm * gearbox ratio. | Used |

## 2. J2 sizing at 75 percent gearbox efficiency

| Case | Gearbox ratio | Motor speed at 30 deg/s joint speed | Required motor torque | Percent of 0.80 N.m holding torque | Screen |
|---|---:|---:|---:|---:|---|
| Current no counterbalance | 50:1 | 250 rpm | 0.670 N.m | 83.7 percent | Reject for now |
| Current no counterbalance | 60:1 | 300 rpm | 0.558 N.m | 69.8 percent | Risky without torque-speed curve |
| Current no counterbalance | 80:1 | 400 rpm | 0.419 N.m | 52.3 percent | Risky without torque-speed curve |
| Current no counterbalance | 100:1 | 500 rpm | 0.335 N.m | 41.9 percent | Conditional target |
| Current no counterbalance | 120:1 | 600 rpm | 0.279 N.m | 34.9 percent | Good torque margin, check speed/backlash |
| 5 N.m counterbalance | 50:1 | 250 rpm | 0.451 N.m | 56.4 percent | Risky without torque-speed curve |
| 5 N.m counterbalance | 60:1 | 300 rpm | 0.376 N.m | 47.0 percent | Conditional target |
| 5 N.m counterbalance | 80:1 | 400 rpm | 0.282 N.m | 35.2 percent | Conditional target |
| 5 N.m counterbalance | 100:1 | 500 rpm | 0.225 N.m | 28.2 percent | Good torque margin, check speed/backlash |
| 8 N.m counterbalance | 50:1 | 250 rpm | 0.320 N.m | 39.9 percent | Conditional target |
| 8 N.m counterbalance | 60:1 | 300 rpm | 0.266 N.m | 33.3 percent | Good torque margin, check speed/backlash |
| 8 N.m counterbalance | 80:1 | 400 rpm | 0.200 N.m | 25.0 percent | Good torque margin, check speed/backlash |
| Mass combo + 5 N.m counterbalance | 50:1 | 250 rpm | 0.346 N.m | 43.2 percent | Conditional target |
| Mass combo + 5 N.m counterbalance | 60:1 | 300 rpm | 0.288 N.m | 36.0 percent | Conditional target |
| Mass combo + 5 N.m counterbalance | 80:1 | 400 rpm | 0.216 N.m | 27.0 percent | Good torque margin, check speed/backlash |

## 3. Recommended ratio windows

| Case | Recommended ratio window | Reason | Status |
|---|---|---|---|
| Current no counterbalance | 100:1 to 120:1 | 80:1 asks for about 52 percent of holding torque at 400 rpm. 100:1 drops this to about 42 percent at 500 rpm. | Conditional |
| 5 N.m counterbalance | 80:1 to 100:1 | 60:1 asks for about 47 percent of holding torque. 80:1 is more believable, 100:1 has more torque margin. | Better |
| 8 N.m counterbalance | 50:1 to 80:1 | 50:1 asks for about 40 percent of holding torque at 250 rpm; 60:1 asks for about 33 percent at 300 rpm. | Preferred if counterbalance fits |
| Mass combo + 5 N.m counterbalance | 50:1 to 80:1 | Similar load level to the 8 N.m counterbalance case while using a smaller counterbalance. | Best balanced |

## 4. Interpretation

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Is current 25.1 N.m J2 torque fine? | It is possible, but it forces J2 toward a high-ratio gearbox around 100:1 to 120:1. | Risky but workable |
| Why not use 50:1 or 60:1 now? | At the current torque level they require about 70-84 percent of holding torque before checking the real torque-speed curve. That is too close for a moving joint. | Not recommended |
| Why counterbalance helps | Counterbalance lowers the output torque demand, so the same motor can use a lower ratio or keep more torque margin. | Recommended |
| Best current direction | Use a J2 counterbalance and then target roughly 50:1 to 80:1 instead of forcing 100:1 to 120:1. | Recommended |

## 5. Required before final gearbox selection

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Stepper torque-speed curve | Need motor torque at approximately 250-600 rpm with the selected driver voltage/current. | Required |
| Gearbox efficiency | Need actual efficiency at load and speed. | Required |
| Backlash | Need backlash small enough to meet TCP repeatability. | Required |
| Output bearing/radial load | Need gearbox output support or separate bearing support for shoulder loads. | Required |
| Gearbox mass | Need actual mass because J2 gearbox placement affects base/shoulder packaging and J3 gearbox mass affects shoulder torque. | Required |

Generated numerical summary: `j2_gearbox_sizing_check_summary.json`  
Calculation source file: `j2_gearbox_sizing_check.ps1`
