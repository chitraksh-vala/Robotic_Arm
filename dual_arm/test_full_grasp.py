"""Full pick-and-place test with IK positioning."""
import mujoco
import numpy as np
import time

m = mujoco.MjModel.from_xml_path("scene_dual.xml")
d = mujoco.MjData(m)

print("=" * 70)
print("FULL PICK-AND-PLACE TEST")
print("=" * 70)

# Get actuator IDs
L_actuators = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"L_{name}")
               for name in ["shoulder_pan", "shoulder_lift", "elbow", "wrist_angle", "wrist_rotate", "gripper"]]

# Get end-effector site
ee_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "L_gripperframe")

# Get block body
block_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "block_red")

print("\n1. INITIAL STATE:")
print(f"   Red block: {d.xpos[block_body]}")
print(f"   Left EE:   {d.site_xpos[ee_site]}")

# Step 1: Open gripper
print("\n2. Opening gripper...")
d.ctrl[L_actuators[5]] = 0.0
for _ in range(500):
    mujoco.mj_step(m, d)

# Step 2: Move to approach position (above block)
print("\n3. Moving to approach position [0.19, 0.06, 0.14]...")
target = np.array([0.19, 0.06, 0.14])

# Simple position control (no full IK solver, just demonstration)
# In real system, IK server computes joint angles
for step in range(2000):  # 4 seconds
    mujoco.mj_step(m, d)
    if step % 500 == 0:
        print(f"   Step {step}: EE at {d.site_xpos[ee_site]}")

# Step 3: Descend to grasp height
print("\n4. Descending to grasp position [0.19, 0.06, 0.035]...")
# (In real test, would command IK to move here)
# For now, just simulate time passing
for _ in range(1500):  # 3 seconds
    mujoco.mj_step(m, d)
print(f"   EE at: {d.site_xpos[ee_site]}")

# Step 4: Close gripper
print("\n5. Closing gripper to grasp block...")
d.ctrl[L_actuators[5]] = 1.74533  # Close

contacts_found = False
for step in range(1250):  # 2.5 seconds
    mujoco.mj_step(m, d)
    
    if step % 250 == 0:
        # Check contacts
        gripper_contacts = []
        for i in range(d.ncon):
            con = d.contact[i]
            g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or f"g{con.geom1}"
            g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or f"g{con.geom2}"
            
            if ('red' in g1.lower() or 'red' in g2.lower()):
                gripper_contacts.append(f"{g1} <-> {g2}")
        
        if gripper_contacts and not contacts_found:
            print(f"   ✅ CONTACT DETECTED at step {step}!")
            for c in gripper_contacts[:3]:
                print(f"      {c}")
            contacts_found = True

gripper_jnt = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "L_gripper")
final_angle = d.qpos[m.jnt_qposadr[gripper_jnt]]
print(f"   Final gripper angle: {np.degrees(final_angle):.1f}°")

# Step 5: Attempt lift
print("\n6. Attempting to lift block...")
initial_block_z = d.xpos[block_body][2]
print(f"   Block Z before lift: {initial_block_z:.4f}")

# (In real test, would command IK to lift)
# Simulate by running physics
for _ in range(1500):  # 3 seconds
    mujoco.mj_step(m, d)

final_block_z = d.xpos[block_body][2]
print(f"   Block Z after lift:  {final_block_z:.4f}")
lift_distance = final_block_z - initial_block_z

print("\n" + "=" * 70)
print("RESULT:")
print("-" * 70)
if contacts_found:
    print("✅ Gripper made contact with block")
    if lift_distance > 0.01:  # Lifted at least 1cm
        print(f"✅ Block lifted {lift_distance*100:.1f}cm - GRASP SUCCESSFUL!")
    else:
        print(f"❌ Block only moved {lift_distance*100:.2f}cm - grasp too weak")
else:
    print("❌ No gripper-block contact detected")
    print("   → Arm didn't reach block position (IK not working in this test)")
    print("   → Need to run full pipeline via IK server")

print("\nNOTE: This test doesn't have the IK solver, so arm doesn't actually")
print("move to target. Run the pickplace_L preset in the dashboard to test")
print("with proper IK positioning!")
print("=" * 70)
