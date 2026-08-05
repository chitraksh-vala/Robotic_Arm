# SO-ARM101 Dual Arm — Startup Guide

## Files
| File | Purpose |
|---|---|
| `scene_dual.xml` | MuJoCo scene: 2 arms + table + 3 coloured blocks |
| `dual_ik_server.py` | IK server (port 7842) |
| `dual_dashboard.html` | Browser dashboard |

## Start

```bash
# Terminal 1 — sim server
python dual_ik_server.py --viewer     # with interactive viewer
# or
python dual_ik_server.py              # headless, use dashboard PNG stream

# Terminal 2 — dashboard HTTP server
python -m http.server 8000

# Open browser
http://127.0.0.1:8000/dual_dashboard.html
```

## Layout

```
Left arm base:   pos = (0, +0.25, 0)   ← blue IK target marker
Right arm base:  pos = (0, -0.25, 0)   ← orange IK target marker
Shared workspace:  x ≈ 0.15–0.35,  y ≈ −0.15 to +0.15
Table surface:   z = 0.005
Block tops:      z ≈ 0.055 (block half-height = 0.025, resting on table)
```

## API quick reference

```bash
# Move left arm
curl -X POST http://127.0.0.1:7842/ik \
  -H 'Content-Type: application/json' \
  -d '{"arm":"L","x":0.22,"y":0.06,"z":0.15,"duration":2.0}'

# Move right arm
curl -X POST http://127.0.0.1:7842/ik \
  -d '{"arm":"R","x":0.22,"y":-0.06,"z":0.15,"duration":2.0}'

# Close left gripper
curl -X POST http://127.0.0.1:7842/gripper -d '{"arm":"L","value":1.0}'

# Home both
curl -X POST http://127.0.0.1:7842/home -d '{"arm":"both"}'

# Reset blocks
curl -X POST http://127.0.0.1:7842/reset_blocks

# Run a dual pipeline step
curl -X POST http://127.0.0.1:7842/pipeline -d '{
  "steps": [
    {"arm":"L","type":"home"},
    {"arm":"R","type":"home"},
    {"arm":"L","type":"move_ik","x":0.22,"y":0.06,"z":0.15,"duration":2.0,"settle_time":0.8},
    {"arm":"L","type":"gripper","value":1.0,"settle_time":1.0}
  ]
}'
```

## Block stacking pipeline

The dashboard "📦 Stack 3 Blocks" preset does:
1. Left arm picks **red** (y=+0.06) → places at stack zone (0.30, 0, table)
2. Right arm picks **green** (y=0) → stacks on red (z + 0.05)
3. Left arm picks **blue** (y=−0.06) → stacks on green (z + 0.10)

After running, use **🔄 Reset Blocks** to put blocks back.

## Adjusting arm spacing

Edit `scene_dual.xml`, lines:
```xml
<body name="L_base" pos="0 0.25 0" ...>   ← change 0.25
<body name="R_base" pos="0 -0.25 0" ...>  ← change -0.25
```
500mm centre-to-centre is the default.
Measure your actual table setup and update accordingly.

## Parallel pipeline steps

Mark steps with `"parallel": true` to run them concurrently with the next step:

```json
{"arm":"L","type":"move_ik","x":0.22,"y":0.06,"z":0.15,"duration":2.0,"parallel":true},
{"arm":"R","type":"move_ik","x":0.22,"y":-0.06,"z":0.15,"duration":2.0}
```

Both arms move simultaneously. The server waits for both to finish before continuing.
