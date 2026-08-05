"""Verify gripper collision box positions."""
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("scene_dual.xml")
d = mujoco.MjData(m)

print("=" * 70)
print("GRIPPER COLLISION GEOMETRY VERIFICATION")
print("=" * 70)

# Step physics once to update positions
mujoco.mj_step(m, d)

# Get gripper geoms
geom_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(m.ngeom)]

print("\nLEFT GRIPPER COLLISION BOXES:")
for i, name in enumerate(geom_names):
    if name and 'L_' in name:
        geom_type = m.geom_type[i]
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            contype = m.geom_contype[i]
            if contype == 8:  # Left gripper collision
                pos = d.geom_xpos[i]
                size = m.geom_size[i]
                print(f"  {name or f'geom_{i}'}")
                print(f"    Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
                print(f"    Size: {size}")
                print(f"    Contype: {contype}")

print("\nRIGHT GRIPPER COLLISION BOXES:")
for i, name in enumerate(geom_names):
    if name and 'R_' in name:
        geom_type = m.geom_type[i]
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            contype = m.geom_contype[i]
            if contype == 16:  # Right gripper collision
                pos = d.geom_xpos[i]
                size = m.geom_size[i]
                print(f"  {name or f'geom_{i}'}")
                print(f"    Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
                print(f"    Size: {size}")
                print(f"    Contype: {contype}")

# Get end-effector positions for reference
L_ee_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "L_gripperframe")
R_ee_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "R_gripperframe")

print("\nEND-EFFECTOR POSITIONS (for reference):")
print(f"  L_gripperframe: [{d.site_xpos[L_ee_site][0]:.4f}, {d.site_xpos[L_ee_site][1]:.4f}, {d.site_xpos[L_ee_site][2]:.4f}]")
print(f"  R_gripperframe: [{d.site_xpos[R_ee_site][0]:.4f}, {d.site_xpos[R_ee_site][1]:.4f}, {d.site_xpos[R_ee_site][2]:.4f}]")

# Close grippers to see collision boxes move
print("\nCLOSING GRIPPERS...")
L_gripper_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "L_gripper")
R_gripper_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "R_gripper")

d.ctrl[L_gripper_act] = 1.74533
d.ctrl[R_gripper_act] = 1.74533

for _ in range(1000):  # 2 seconds
    mujoco.mj_step(m, d)

print("\nAFTER CLOSING:")
print("LEFT GRIPPER COLLISION BOXES:")
for i, name in enumerate(geom_names):
    if name and 'L_' in name:
        geom_type = m.geom_type[i]
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            contype = m.geom_contype[i]
            if contype == 8:
                pos = d.geom_xpos[i]
                print(f"  {name or f'geom_{i}'}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

print("\nRIGHT GRIPPER COLLISION BOXES:")
for i, name in enumerate(geom_names):
    if name and 'R_' in name:
        geom_type = m.geom_type[i]
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            contype = m.geom_contype[i]
            if contype == 16:
                pos = d.geom_xpos[i]
                print(f"  {name or f'geom_{i}'}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

print("\n" + "=" * 70)
print("✅ Check that collision boxes are at gripper tips, not bases!")
print("=" * 70)
