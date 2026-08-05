"""
Diagnostic script to understand IK and arm kinematics
"""
import mujoco
import numpy as np

SCENE = "scene_dual.xml"

m = mujoco.MjModel.from_xml_path(SCENE)
d = mujoco.MjData(m)
mujoco.mj_resetData(m, d)
mujoco.mj_forward(m, d)

print("=" * 60)
print("ARM KINEMATICS DIAGNOSTIC")
print("=" * 60)

for arm in ["L", "R"]:
    print(f"\n{arm} ARM:")
    print("-" * 40)
    
    # Get joint positions
    joints = [f"{arm}_shoulder_pan", f"{arm}_shoulder_lift", f"{arm}_elbow_flex", 
              f"{arm}_wrist_flex", f"{arm}_wrist_roll"]
    
    print("\nJoint positions at 'home' (all zeros):")
    for jname in joints:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        qadr = m.joint(jid).qposadr[0]
        qval = d.qpos[qadr]
        jrange = m.joint(jid).range
        print(f"  {jname:20s}: {qval:7.3f} rad  ({qval*180/np.pi:6.1f}°)  range=[{jrange[0]:6.3f}, {jrange[1]:6.3f}]")
    
    # Get EE position
    ee_site = f"{arm}_gripperframe"
    ee_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, ee_site)
    ee_pos = d.site_xpos[ee_id]
    print(f"\nEnd-effector position: [{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}]")
    
    # Get base position
    base_body = f"{arm}_base"
    base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, base_body)
    base_pos = d.xpos[base_id]
    print(f"Base position:         [{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}]")
    
    # Distance from base
    dist = np.linalg.norm(ee_pos - base_pos)
    print(f"Reach (base to EE):    {dist:.4f} m")

print("\n" + "=" * 60)
print("\nNow testing IK to a simple target...")
print("=" * 60)

from dual_ik_server import DLSIK, ARMS

for arm in ["L"]:
    print(f"\n{arm} ARM IK TEST:")
    print("-" * 40)
    
    ik = DLSIK(m, ARMS[arm]["ee_site"], ARMS[arm]["joints"])
    
    # Test 1: Can we reach the current home position?
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    ee_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, ARMS[arm]["ee_site"])
    home_ee = d.site_xpos[ee_id].copy()
    
    print(f"\nTest 1: Reach current home position {[f'{v:.3f}' for v in home_ee]}")
    q, err, iters = ik.solve(home_ee, q0=[0,0,0,0,0])
    print(f"  Result: q={[f'{v:.3f}' for v in q]}, err={err*1000:.1f}mm, iters={iters}")
    
    # Test 2: Move down a bit
    target = [home_ee[0], home_ee[1], home_ee[2] - 0.10]
    print(f"\nTest 2: Move DOWN 10cm to {[f'{v:.3f}' for v in target]}")
    q, err, iters = ik.solve(target, q0=[0,0,0,0,0])
    print(f"  Result: q={[f'{v:.3f}' for v in q]}, err={err*1000:.1f}mm, iters={iters}")
    
    # Test 3: Move to side (toward center)
    target = [home_ee[0], home_ee[1] - 0.10, home_ee[2]]
    print(f"\nTest 3: Move INWARD 10cm to {[f'{v:.3f}' for v in target]}")
    q, err, iters = ik.solve(target, q0=[0,0,0,0,0])
    print(f"  Result: q={[f'{v:.3f}' for v in q]}, err={err*1000:.1f}mm, iters={iters}")

print("\n" + "=" * 60)
