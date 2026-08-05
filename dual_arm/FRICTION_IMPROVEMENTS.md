# FRICTION AND GRIP IMPROVEMENTS

## Changes Made to Fix Block Pushing Issue

### Problem
- Gripper was pushing blocks instead of picking them up
- Insufficient friction between gripper jaws and blocks
- Cube shape is difficult to grip with single-jaw gripper

### Solutions Implemented

#### 1. **Increased Friction Dramatically**
   - **Blocks**: friction 0.8 → 2.0 (2.5x increase)
   - **Gripper collision boxes**: Added friction="2.0 0.05 0.005"
   - High sliding friction prevents pushing/slipping

#### 2. **Made Blocks Rectangular (Taller)**
   - Size: 40×40×40mm → 40×40×50mm (cube → rectangular prism)
   - Height increased by 25% (easier to wrap gripper around)
   - Better grip with more surface area

#### 3. **Adjusted Approach Angle**
   - X position: 0.19 → 0.18 (approach from slightly closer)
   - Reduces likelihood of pushing block forward
   - Gripper wraps around block better

#### 4. **Updated Heights for Taller Blocks**
   - Grasp height: z=0.030 → z=0.035 (center of 50mm tall block)
   - Stack 2nd: z=0.070 → z=0.085
   - Stack 3rd: z=0.110 → z=0.135

## New Block Specifications
```
Shape: Rectangular box (40mm × 40mm × 50mm)
Mass: 40g
Position on table: z=0.03 (center height)
Top surface: z=0.055
Bottom surface: z=0.005 (on table)
Friction coefficient: 2.0 (very high)
```

## New Grasp Parameters
```
Approach: x=0.18 (1cm closer than block center at 0.22)
Grasp height: z=0.035 (middle of block)
Wrist rotation: 1.5708 rad (90°) for left, -1.5708 for right
```

## Stack Heights (for 50mm blocks)
```
1st block center: z=0.035 (0.005 table + 0.030 half-height)
2nd block center: z=0.085 (0.035 + 0.050)
3rd block center: z=0.135 (0.085 + 0.050)
```

## Physics Explanation
- **Friction coefficient 2.0** means static friction force = 2.0 × normal force
- This is very high (rubber on rubber ≈ 1.0-1.5)
- Prevents blocks from sliding when gripper closes
- Gripper can now "bite" into block position instead of pushing

## To Test
1. Restart IK server: `python dual_ik_server.py`
2. Open dashboard: `http://localhost:7842/dual_dashboard.html`
3. Run `pickplace_L` - should now grip without pushing!
4. Blocks should be taller and easier to see being gripped

## Expected Behavior
✅ Gripper approaches from slightly forward position
✅ Jaws close around block without pushing
✅ High friction prevents slipping
✅ Taller block provides better grip surface
✅ Block lifts cleanly off table
