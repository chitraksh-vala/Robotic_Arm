"""Test collision setup in the dual arm scene."""
import mujoco
import numpy as np

print("=" * 60)
print("COLLISION SETUP TEST")
print("=" * 60)

# Load model
m = mujoco.MjModel.from_xml_path("scene_dual.xml")
d = mujoco.MjData(m)

print(f"\n✓ Model loaded successfully")
print(f"  Bodies: {m.nbody}")
print(f"  Geoms: {m.ngeom}")
print(f"  Joints: {m.njnt}")

# Check collision geoms
print(f"\nCollision Geoms:")
print(f"{'Name':<30} {'Type':<12} {'contype':<8} {'conaffinity':<12} {'Group'}")
print("-" * 80)

for i in range(m.ngeom):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
    if name is None:
        name = f"<unnamed_{i}>"
    gtype = m.geom_type[i]
    contype = m.geom_contype[i]
    conaffinity = m.geom_conaffinity[i]
    group = m.geom_group[i]
    
    # Only show collision geoms (group 3) and blocks/table (group 0)
    if group in [0, 3]:
        type_names = ['PLANE', 'HFIELD', 'SPHERE', 'CAPSULE', 'ELLIPSOID', 'CYLINDER', 'BOX', 'MESH']
        type_str = type_names[gtype] if gtype < len(type_names) else f"TYPE_{gtype}"
        print(f"{name:<30} {type_str:<12} {contype:<8} {conaffinity:<12} {group}")

# Test collision detection
mujoco.mj_step(m, d)
print(f"\n✓ Physics step completed")
print(f"  Active contacts: {d.ncon}")

if d.ncon > 0:
    print(f"\nActive Contacts:")
    for i in range(min(d.ncon, 10)):  # Show first 10
        con = d.contact[i]
        g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or f"geom_{con.geom1}"
        g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or f"geom_{con.geom2}"
        print(f"  {g1:<25} <-> {g2:<25}  dist={con.dist:.4f}")

print("\n" + "=" * 60)
print("Collision Bit Assignments:")
print("-" * 60)
print("Bit 0 (1):   Environment (floor, table, blocks)")
print("Bit 1 (2):   Left arm links")
print("Bit 2 (4):   Right arm links")
print("Bit 3 (8):   Left gripper")
print("Bit 4 (16):  Right gripper")
print("\nExpected interactions:")
print("  • Left arm (2) collides with: env(1), L_arm(2), R_arm(4) = affinity 7")
print("  • Right arm (4) collides with: env(1), L_arm(2), R_arm(4) = affinity 7")
print("  • Left gripper (8) collides with: env(1), R_arm(4) = affinity 5")
print("  • Right gripper (16) collides with: env(1), L_arm(2) = affinity 9")
print("=" * 60)
