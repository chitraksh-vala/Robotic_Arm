"""
Quick test script to verify arm motion works correctly.
Run after starting dual_ik_server.py
"""
import requests
import time
import json

SERVER = "http://127.0.0.1:7842"

def api(method, path, data=None):
    url = f"{SERVER}{path}"
    if method == "GET":
        r = requests.get(url)
    else:
        r = requests.post(url, json=data)
    return r.json()

def main():
    print("Testing dual arm motion...")
    print("=" * 60)
    
    # Get initial state
    print("\n1. Getting initial state...")
    state = api("GET", "/state")
    l_ee = state["arms"]["L"]["ee"]
    r_ee = state["arms"]["R"]["ee"]
    print(f"   Left EE:  [{l_ee[0]:.3f}, {l_ee[1]:.3f}, {l_ee[2]:.3f}]")
    print(f"   Right EE: [{r_ee[0]:.3f}, {r_ee[1]:.3f}, {r_ee[2]:.3f}]")
    print("\n   NOTE: Arms start extended forward (~0.39m from base)")
    print("         Try moving SIDEWAYS or DOWN, not backward!\n")
    
    # Test 1: Move left arm DOWN (keeping X, Y similar)
    target_L = [0.35, 0.25, 0.10]  # Down from [0.391, 0.250, 0.226]
    print(f"\n2. Moving LEFT arm DOWN to {target_L}...")
    result = api("POST", "/ik", {"arm": "L", "x": target_L[0], "y": target_L[1], "z": target_L[2], "duration": 2.0})
    print(f"   IK solved, waiting for motion...")
    time.sleep(2.5)
    
    state = api("GET", "/state")
    l_ee = state["arms"]["L"]["ee"]
    l_err = state['arms']['L']['ik_err_mm']
    print(f"   Left EE now: [{l_ee[0]:.3f}, {l_ee[1]:.3f}, {l_ee[2]:.3f}]")
    print(f"   IK error: {l_err:.2f} mm {'✓ GOOD' if l_err < 10 else '✗ FAILED'}")
    
    # Test 2: Move right arm DOWN 
    target_R = [0.35, -0.25, 0.10]  # Symmetric for right
    print(f"\n3. Moving RIGHT arm DOWN to {target_R}...")
    result = api("POST", "/ik", {"arm": "R", "x": target_R[0], "y": target_R[1], "z": target_R[2], "duration": 2.0})
    print(f"   IK solved, waiting for motion...")
    time.sleep(2.5)
    
    state = api("GET", "/state")
    r_ee = state["arms"]["R"]["ee"]
    r_err = state['arms']['R']['ik_err_mm']
    print(f"   Right EE now: [{r_ee[0]:.3f}, {r_ee[1]:.3f}, {r_ee[2]:.3f}]")
    print(f"   IK error: {r_err:.2f} mm {'✓ GOOD' if r_err < 10 else '✗ FAILED'}")
    
    # Test 3: Move left arm to side (toward center)
    print(f"\n4. Moving LEFT arm toward CENTER [0.35, 0.15, 0.15]...")
    result = api("POST", "/ik", {"arm": "L", "x": 0.35, "y": 0.15, "z": 0.15, "duration": 2.0})
    time.sleep(2.5)
    state = api("GET", "/state")
    l_ee = state["arms"]["L"]["ee"]
    l_err = state['arms']['L']['ik_err_mm']
    print(f"   Left EE now: [{l_ee[0]:.3f}, {l_ee[1]:.3f}, {l_ee[2]:.3f}]")
    print(f"   IK error: {l_err:.2f} mm {'✓ GOOD' if l_err < 10 else '✗ FAILED'}")
    
    # Test 4: Gripper test
    print("\n5. Testing LEFT gripper...")
    api("POST", "/gripper", {"arm": "L", "value": 1.0})
    print("   Closing...")
    time.sleep(1.0)
    api("POST", "/gripper", {"arm": "L", "value": 0.0})
    print("   Opening...")
    time.sleep(1.0)
    
    # Test 5: Home both
    print("\n6. Homing both arms...")
    api("POST", "/home", {"arm": "both"})
    time.sleep(1.5)
    state = api("GET", "/state")
    l_ee = state["arms"]["L"]["ee"]
    r_ee = state["arms"]["R"]["ee"]
    print(f"   Left EE:  [{l_ee[0]:.3f}, {l_ee[1]:.3f}, {l_ee[2]:.3f}]")
    print(f"   Right EE: [{r_ee[0]:.3f}, {r_ee[1]:.3f}, {r_ee[2]:.3f}]")
    
    print("\n" + "=" * 60)
    if l_err < 10 or r_err < 10:
        print("✓ At least some targets reachable!")
    else:
        print("✗ IK not converging - targets may be unreachable")
        print("  Try using the dashboard to find working positions")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("Make sure dual_ik_server.py is running on port 7842")
