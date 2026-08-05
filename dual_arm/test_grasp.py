"""Test if gripper can grasp a block."""
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("scene_dual.xml")
d = mujoco.MjData(m)

print("=" * 70)
print("GRIPPER GRASP TEST")
print("=" * 70)

# Position left arm near red block
L_gripper_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "L_gripper")
L_gripper_jnt = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "L_gripper")

# Move gripper to block position (simulating after IK movement)
# Red block at [0.22, 0.06, 0.03], we want to grasp from x=0.19
ee_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "L_gripperframe")

print("\n1. Initial state:")
print(f"   Red block: [0.22, 0.06, 0.03]")
print(f"   Target grasp: [0.19, 0.06, 0.04]")

# Open gripper
d.ctrl[L_gripper_act] = 0.0  # Open
for _ in range(500):
    mujoco.mj_step(m, d)

print(f"\n2. Gripper opened: {d.qpos[m.jnt_qposadr[L_gripper_jnt]]:.3f} rad")

# Close gripper
print("\n3. Closing gripper around block...")
d.ctrl[L_gripper_act] = 1.74533  # Close

contacts_during_close = []
for step in range(1000):
    mujoco.mj_step(m, d)
    
    # Check for contacts involving gripper or block
    if step % 100 == 0:
        ncon = d.ncon
        gripper_block_contacts = []
        
        for i in range(ncon):
            con = d.contact[i]
            g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or f"g{con.geom1}"
            g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or f"g{con.geom2}"
            
            # Check if contact involves red block
            if 'red' in g1.lower() or 'red' in g2.lower():
                gripper_block_contacts.append(f"{g1} <-> {g2}")
        
        angle = d.qpos[m.jnt_qposadr[L_gripper_jnt]]
        print(f"   Step {step}: angle={np.degrees(angle):.1f}°, contacts={len(gripper_block_contacts)}")
        if gripper_block_contacts:
            for c in gripper_block_contacts[:2]:
                print(f"      {c}")

# Check final state
final_angle = d.qpos[m.jnt_qposadr[L_gripper_jnt]]
print(f"\n4. Final gripper angle: {final_angle:.3f} rad ({np.degrees(final_angle):.1f}°)")

# Check if block moved
block_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "block_red")
block_pos = d.xpos[block_body]
print(f"   Red block position: [{block_pos[0]:.4f}, {block_pos[1]:.4f}, {block_pos[2]:.4f}]")
print(f"   Block moved: {np.linalg.norm(block_pos - np.array([0.22, 0.06, 0.03])) > 0.001}")

# Check contacts in final state
print("\n5. Final contacts:")
has_gripper_block_contact = False
for i in range(d.ncon):
    con = d.contact[i]
    g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or f"g{con.geom1}"
    g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or f"g{con.geom2}"
    
    if 'red' in g1.lower() or 'red' in g2.lower():
        print(f"   {g1} <-> {g2}")
        has_gripper_block_contact = True

print("\n" + "=" * 70)
print("DIAGNOSIS:")
print("-" * 70)
if has_gripper_block_contact:
    if np.degrees(final_angle) < 50:
        print("❌ Gripper closed before reaching block (closed too early)")
    else:
        print("✅ Gripper is in contact with block")
        if np.linalg.norm(block_pos - np.array([0.22, 0.06, 0.03])) < 0.001:
            print("❌ But block didn't move - gripper not generating grip force")
        else:
            print("✅ Block moved - grasp successful!")
else:
    print("❌ NO CONTACT between gripper and block")
    print("   → Gripper has no collision geometry")
    print("   → Visual meshes have contype=0 (no collision)")
    print("   → Need to add collision geoms to gripper jaws")

print("=" * 70)
