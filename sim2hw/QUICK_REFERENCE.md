# Gripper Orientation Control - Quick Reference

## What Changed

### Before
- Gripper orientation was perpendicular to floor (fingers pointing down/up)
- IK solver only controlled position (x, y, z)
- Pick-and-place had suboptimal grasp approach

### After
- Gripper fingers can be horizontal (parallel to floor) 
- `wrist_angle` parameter controls gripper rotation via wrist_roll joint
- Pick-and-place uses optimal horizontal finger approach at cylinder middle height

## Visual Representation

```
BEFORE (fingers vertical):          AFTER (fingers horizontal with wrist_angle=1.57):
                                
    Gripper fingers                  Gripper fingers
         | |                         rotated 90°
         | |                              ___
    ↓   (   )                       ↓   |   |
                                        |___|
   -----                            -----
   | O |  ← Cylinder                | O |  ← Cylinder  
   -----                            -----
```

## Key Parameters

### Cylinder Specs (from scene_full.xml)
- Position: (0.25, 0.0, 0.02)
- Radius: 0.015m
- Half-height: 0.02m
- **Total height: 0.04m**
- **Bottom at z=0.0, Middle at z=0.02, Top at z=0.04**

### Wrist Angle Values (wrist_roll joint)
- `π/2 ≈ 1.57 rad`: Horizontal gripper fingers, parallel to floor (PICK-AND-PLACE)
- `0 rad`: Default IK orientation (fingers usually vertical)
- `-π/2 ≈ -1.57 rad`: Horizontal gripper fingers, opposite direction

**Important**: `wrist_angle` controls **wrist_roll** (rotation around gripper axis), not wrist_flex (tilt)

## Modified Files

1. **ik_server.py**
   - `solve_ik()`: Added `wrist_angle` parameter controlling wrist_roll (index 4)
   - Pipeline execution: Extracts `wrist_angle` from steps
   - HTTP `/ik` endpoint: Accepts `wrist_angle` in request

2. **so101_dashboard.html**
   - Pick-and-place preset: All moves now use `wrist_angle: 1.57`
   - Updated z-coordinates to target cylinder middle (z=0.02)

## Usage in Pipeline Steps

```javascript
{
  type: 'move_ik',
  label: 'Approach object horizontally',
  x: 0.25,
  y: 0.0,
  z: 0.02,           // Middle of cylinder
  wrist_angle: 1.57,  // 90° = horizontal gripper fingers
  duration: 1.5,
  settle_time: 0.8
}
```

## Try It Now!

1. Start IK server:
   ```bash
   python ik_server.py --viewer
   ```

2. Start dashboard server:
   ```bash
   python -m http.server 8000
   ```

3. Open browser: `http://127.0.0.1:8000/so101_dashboard.html`

4. Click "📦 Pick & Place" preset button

5. Watch the arm approach with **horizontal gripper fingers** at **middle height**!

## Optional: Hardware Testing

If you have the physical arm connected:

```bash
# Terminal 2 (hardware bridge)
python hw_bridge.py --port COM4

# Then click "Connect HW" in the dashboard
# The real arm will mirror the simulation with horizontal finger approach
```

## Implementation Details

The wrist angle is applied **after** the IK solve:
1. IK solver finds joint angles for position (x, y, z)
2. If `wrist_angle` is specified, override `wrist_roll` (joint index 4)
3. Recalculate position error with new wrist angle
4. Interpolate smoothly to the target configuration

Joint Structure:
- ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
- Index 4 = wrist_roll (controls rotation of gripper around its axis)

This approach:
- ✅ Maintains position accuracy
- ✅ Allows per-move orientation control
- ✅ Backward compatible (omit parameter for default behavior)
- ✅ Smooth motion via interpolation

## Troubleshooting

**Issue**: Gripper fingers still look vertical
- **Check**: Make sure `wrist_angle: 1.57` is in the step (not -1.57)
- **Verify**: Look for "wrist_angle=1.570" in console output
- **Note**: wrist_roll (not wrist_flex) controls finger rotation

**Issue**: Arm can't reach position
- **Check**: Wrist angle constraint might make position unreachable
- **Try**: Adjust z-height or wrist angle value

**Issue**: Jerky motion
- **Check**: `duration` and `settle_time` values
- **Recommended**: duration ≥ 1.5s for wrist angle changes

**Issue**: Wrong rotation direction
- **Try**: Use `-1.57` instead of `1.57` (or vice versa)
- **Note**: The optimal angle depends on how the gripper is mounted
