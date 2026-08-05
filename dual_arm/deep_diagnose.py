"""Deep dive into why the gripper can't close."""
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("scene_dual.xml")
d = mujoco.MjData(m)

print("=" * 70)
print("GRIPPER ACTUATION DEEP DIVE")
print("=" * 70)

# Get gripper joint and actuator info
jnt_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "L_gripper")
act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "L_gripper")

print(f"\nL_gripper joint (id={jnt_id}):")
qpos_addr = m.jnt_qposadr[jnt_id]
print(f"  qpos address: {qpos_addr}")
print(f"  Range: [{m.jnt_range[jnt_id][0]:.4f}, {m.jnt_range[jnt_id][1]:.4f}]")
print(f"  Damping: {m.dof_damping[jnt_id]:.4f}")
print(f"  Armature: {m.dof_armature[jnt_id]:.4f}")
print(f"  Friction loss: {m.dof_frictionloss[jnt_id]:.4f}")

print(f"\nL_gripper actuator (id={act_id}):")
print(f"  Control range: [{m.actuator_ctrlrange[act_id][0]:.4f}, {m.actuator_ctrlrange[act_id][1]:.4f}]")
print(f"  Force range: [{m.actuator_forcerange[act_id][0]:.4f}, {m.actuator_forcerange[act_id][1]:.4f}]")
print(f"  Gain (kp): {m.actuator_gainprm[act_id][0]:.4f}")
print(f"  Bias (kv): {m.actuator_biasprm[act_id][0]:.4f}")

# Try closing with different timesteps and durations
print("\n" + "=" * 70)
print("TESTING DIFFERENT SIMULATION DURATIONS")
print("=" * 70)

for duration_sec in [0.5, 1.0, 2.0, 5.0]:
    d2 = mujoco.MjData(m)
    d2.ctrl[act_id] = 1.74533  # Command close
    
    n_steps = int(duration_sec / m.opt.timestep)
    for _ in range(n_steps):
        mujoco.mj_step(m, d2)
    
    actual = d2.qpos[qpos_addr]
    print(f"  After {duration_sec:.1f}s ({n_steps} steps): {actual:.4f} rad ({np.degrees(actual):.1f}°)")

# Check for contacts during gripper closing
print("\n" + "=" * 70)
print("CHECKING CONTACTS DURING CLOSING")
print("=" * 70)

d3 = mujoco.MjData(m)
d3.ctrl[act_id] = 1.74533

contact_history = []
for step in range(0, 2500, 100):  # Check every 100 steps for 5 seconds
    for _ in range(100):
        mujoco.mj_step(m, d3)
    
    actual = d3.qpos[qpos_addr]
    ncon = d3.ncon
    
    # Check if any contacts involve gripper geoms
    gripper_contacts = []
    for i in range(ncon):
        con = d3.contact[i]
        g1_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or f"geom_{con.geom1}"
        g2_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or f"geom_{con.geom2}"
        
        # Check if either geom belongs to gripper body or moving jaw
        if any(x in g1_name.lower() or x in g2_name.lower() for x in ['gripper', 'jaw', 'wrist', 'lower_arm']):
            gripper_contacts.append(f"{g1_name} <-> {g2_name}")
    
    if gripper_contacts or step <= 500:
        print(f"\nStep {step}: qpos={actual:.4f} rad ({np.degrees(actual):.1f}°), contacts={len(gripper_contacts)}")
        for gc in gripper_contacts[:3]:  # Show first 3
            print(f"    {gc}")

# Check applied forces
print("\n" + "=" * 70)
print("ACTUATOR FORCE ANALYSIS")
print("=" * 70)

d4 = mujoco.MjData(m)
d4.ctrl[act_id] = 1.74533

for _ in range(2500):
    mujoco.mj_step(m, d4)

actual = d4.qpos[qpos_addr]
qfrc_actuator = d4.qfrc_actuator[jnt_id]
qfrc_passive = d4.qfrc_passive[jnt_id]
qfrc_constraint = d4.qfrc_constraint[jnt_id]

print(f"\nFinal position: {actual:.4f} rad ({np.degrees(actual):.1f}°)")
print(f"Target position: 1.7453 rad (100.0°)")
print(f"Position error: {1.7453 - actual:.4f} rad ({np.degrees(1.7453 - actual):.1f}°)")
print(f"\nForce breakdown:")
print(f"  Actuator force: {qfrc_actuator:.6f}")
print(f"  Passive force: {qfrc_passive:.6f}")
print(f"  Constraint force: {qfrc_constraint:.6f}")
print(f"  Total: {qfrc_actuator + qfrc_passive + qfrc_constraint:.6f}")

print("\n" + "=" * 70)
print("CONCLUSION")
print("=" * 70)
if abs(qfrc_constraint) > 0.001:
    print("❌ Constraint force is NON-ZERO - something is colliding!")
elif abs(qfrc_passive) > abs(qfrc_actuator):
    print("❌ Passive forces (damping/friction) exceed actuator force")
else:
    print("❓ Unknown issue preventing gripper closure")
