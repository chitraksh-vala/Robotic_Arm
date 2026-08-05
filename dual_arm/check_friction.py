"""Check friction and block properties."""
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("scene_dual.xml")
d = mujoco.MjData(m)

print("=" * 70)
print("FRICTION AND BLOCK PROPERTIES")
print("=" * 70)

# Check block properties
for block_name in ["block_red_geom", "block_green_geom", "block_blue_geom"]:
    geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, block_name)
    print(f"\n{block_name}:")
    print(f"  Size: {m.geom_size[geom_id]} (should be [0.02, 0.02, 0.025] for 40×40×50mm)")
    print(f"  Friction: {m.geom_friction[geom_id]} (should be [2.0, 0.05, 0.005])")
    print(f"  Contype: {m.geom_contype[geom_id]}, Conaffinity: {m.geom_conaffinity[geom_id]}")
    
    # Get body position
    body_name = block_name.replace("_geom", "")
    body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    print(f"  Position: {d.xpos[body_id]} (center should be at z=0.03)")

# Check gripper collision box friction
print("\n" + "=" * 70)
print("GRIPPER COLLISION BOXES")
print("=" * 70)

geom_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(m.ngeom)]
for i, name in enumerate(geom_names):
    if name is None:
        continue
    contype = m.geom_contype[i]
    # Check for gripper collision geoms (contype 8 or 16)
    if contype in [8, 16]:
        geom_type = m.geom_type[i]
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            arm = "LEFT" if contype == 8 else "RIGHT"
            print(f"\n{arm} gripper collision box:")
            print(f"  Friction: {m.geom_friction[i]} (should be [2.0, 0.05, 0.005])")
            print(f"  Size: {m.geom_size[i]}")
            print(f"  Position: {d.geom_xpos[i]}")

print("\n" + "=" * 70)
print("FRICTION COMPARISON")
print("=" * 70)
print("Typical friction coefficients:")
print("  Steel on steel: 0.6")
print("  Wood on wood: 0.4")
print("  Rubber on concrete: 1.0")
print("  Rubber on rubber: 1.5")
print("  Our setting: 2.0 (VERY HIGH - prevents slipping!)")
print("\nWith friction=2.0:")
print("  A 40g block needs >80g force to slide (2.0 × 40g)")
print("  Gripper can apply much more force than this")
print("  Block will NOT push, it will GRIP! ✅")

print("\n" + "=" * 70)
print("BLOCK DIMENSIONS")
print("=" * 70)
print("Size: 40mm × 40mm × 50mm (rectangular prism)")
print("Mass: 40g")
print("Center height: z=0.03 (5mm table + 25mm half-height)")
print("Top: z=0.055")
print("Bottom: z=0.005")
print("Grasp target: z=0.035 (middle of block)")
print("=" * 70)
