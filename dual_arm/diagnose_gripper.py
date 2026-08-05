"""Diagnose gripper geometry and positioning."""
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("scene_dual.xml")
d = mujoco.MjData(m)

print("=" * 70)
print("GRIPPER GEOMETRY DIAGNOSTIC")
print("=" * 70)

# Find gripper-related geoms
for i in range(m.ngeom):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
    if name and ('gripper' in name.lower() or 'jaw' in name.lower() or 'block' in name.lower()):
        gtype = m.geom_type[i]
        type_names = ['PLANE', 'HFIELD', 'SPHERE', 'CAPSULE', 'ELLIPSOID', 'CYLINDER', 'BOX', 'MESH']
        type_str = type_names[gtype]
        contype = m.geom_contype[i]
        conaffinity = m.geom_conaffinity[i]
        size = m.geom_size[i]
        pos = m.geom_pos[i]
        print(f"\n{name}:")
        print(f"  Type: {type_str}, Size: {size}")
        print(f"  Local pos: {pos}")
        print(f"  contype={contype}, conaffinity={conaffinity}")

# Check gripper joint ranges and current positions
print("\n" + "=" * 70)
print("GRIPPER JOINTS")
print("=" * 70)
for arm in ['L', 'R']:
    jnt_name = f"{arm}_gripper"
    jnt_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
    if jnt_id >= 0:
        qpos_addr = m.jnt_qposadr[jnt_id]
        range_min, range_max = m.jnt_range[jnt_id]
        current = d.qpos[qpos_addr]
        print(f"\n{jnt_name}:")
        print(f"  Range: [{range_min:.4f}, {range_max:.4f}] rad")
        print(f"  Range: [{np.degrees(range_min):.1f}°, {np.degrees(range_max):.1f}°]")
        print(f"  Current: {current:.4f} rad ({np.degrees(current):.1f}°)")
        print(f"  Open (0 rad = 0°), Closed (1.745 rad = 100°)")

# Check EE position
print("\n" + "=" * 70)
print("END-EFFECTOR POSITIONS (at home)")
print("=" * 70)
mujoco.mj_forward(m, d)
for arm in ['L', 'R']:
    site_name = f"{arm}_gripperframe"
    site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id >= 0:
        pos = d.site_xpos[site_id]
        print(f"{arm}_gripperframe: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

# Now simulate closing the gripper and see what happens
print("\n" + "=" * 70)
print("GRIPPER CLOSING TEST")
print("=" * 70)

# Set left gripper to closed position
jnt_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "L_gripper")
act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "L_gripper")
if jnt_id >= 0 and act_id >= 0:
    qpos_addr = m.jnt_qposadr[jnt_id]
    # Command gripper to close
    d.ctrl[act_id] = 1.74533  # 100 degrees (closed)
    
    # Run simulation for 1 second
    for _ in range(500):
        mujoco.mj_step(m, d)
    
    current = d.qpos[qpos_addr]
    print(f"\nAfter closing L_gripper:")
    print(f"  Command: 1.74533 rad (100°)")
    print(f"  Actual: {current:.4f} rad ({np.degrees(current):.1f}°)")
    print(f"  Gripper reached: {abs(current - 1.74533) < 0.1}")

print("\n" + "=" * 70)
print("BLOCK POSITIONS")
print("=" * 70)
for color in ['red', 'green', 'blue']:
    body_name = f"block_{color}"
    body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id >= 0:
        pos = d.xpos[body_id]
        print(f"{body_name}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

print("\nBlock height analysis:")
print(f"  Table surface: z = 0.005")
print(f"  Block size: 0.025m (cube)")
print(f"  Block center on table: z = 0.005 + 0.025 = 0.030")
print(f"  Block top: z = 0.030 + 0.025 = 0.055")
print(f"  Grasp target should be: z ≈ 0.035-0.045 (middle of block)")
