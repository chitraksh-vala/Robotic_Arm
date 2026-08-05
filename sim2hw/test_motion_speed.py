"""
Test script to adjust SO-ARM101 hardware motion speed and smoothness.
Usage: python test_motion_speed.py [--time 1500] [--preset slow]

For STS3215 servos, motion is controlled by Goal_Time (milliseconds).
Higher time = slower, smoother motion.
"""
import argparse
import json
import urllib.request
import time

HW_BRIDGE_URL = "http://127.0.0.1:7843"

def set_motion_time(time_ms):
    """Set motion time in milliseconds (higher = slower, smoother)"""
    req = urllib.request.Request(
        HW_BRIDGE_URL + "/set_motion_time",
        data=json.dumps({"time_ms": time_ms}).encode(),
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    response = urllib.request.urlopen(req, timeout=2)
    result = json.loads(response.read())
    print(f"✓ Motion time set to {result['time_ms']}ms ({result['time_ms']/1000:.1f} seconds)")

def get_params():
    """Get current motion parameters"""
    response = urllib.request.urlopen(HW_BRIDGE_URL + "/motion_params", timeout=2)
    result = json.loads(response.read())
    print(f"Current settings:")
    print(f"  Motion time: {result['time_ms']}ms ({result['time_ms']/1000:.1f}s to reach target)")
    print(f"  (Higher time = slower, smoother motion)")

def test_motion():
    """Send a simple test motion to see the effect"""
    print("\nSending test motion: home → small movement → home")
    
    # Home position
    req = urllib.request.Request(
        HW_BRIDGE_URL + "/home",
        data=json.dumps({}).encode(),
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    urllib.request.urlopen(req, timeout=2)
    print("  Moving to home...")
    time.sleep(3)
    
    # Small test movement
    req = urllib.request.Request(
        HW_BRIDGE_URL + "/move",
        data=json.dumps({
            "joints": {
                "shoulder_pan": 0.3,
                "shoulder_lift": 0.2,
                "elbow_flex": -0.3
            }
        }).encode(),
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    urllib.request.urlopen(req, timeout=2)
    print("  Moving joints...")
    time.sleep(3)
    
    # Back to home
    req = urllib.request.Request(
        HW_BRIDGE_URL + "/home",
        data=json.dumps({}).encode(),
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    urllib.request.urlopen(req, timeout=2)
    print("  Returning to home...")
    time.sleep(3)
    print("✓ Test complete\n")

def main():
    parser = argparse.ArgumentParser(description="Adjust SO-ARM101 motion parameters")
    parser.add_argument("--time", type=int, help="Motion time in milliseconds (100-5000, higher = slower)")
    parser.add_argument("--test", action="store_true", help="Run a test motion")
    parser.add_argument("--preset", choices=["very-slow", "slow", "medium", "fast", "very-fast"],
                       help="Use a preset: very-slow (3000ms), slow (2000ms), medium (1500ms), fast (1000ms), very-fast (500ms)")
    args = parser.parse_args()

    try:
        # Check connection
        response = urllib.request.urlopen(HW_BRIDGE_URL + "/ping", timeout=2)
        result = json.loads(response.read())
        if not result.get("connected"):
            print("⚠  Hardware bridge not connected to arm!")
            return
        
        print("✓ Connected to hardware bridge\n")
        
        # Apply preset if specified
        if args.preset:
            presets = {
                "very-slow": 3000,
                "slow": 2000,
                "medium": 1500,
                "fast": 1000,
                "very-fast": 500
            }
            time_ms = presets[args.preset]
            set_motion_time(time_ms)
        elif args.time is not None:
            # Apply custom time
            set_motion_time(args.time)
        
        print()
        get_params()
        
        # Run test if requested
        if args.test:
            test_motion()
        
        print("\nTip: Try these presets to find your preferred speed:")
        print("  --preset very-slow  (3000ms = 3.0s, very gentle)")
        print("  --preset slow       (2000ms = 2.0s, smooth)")
        print("  --preset medium     (1500ms = 1.5s, balanced, default)")
        print("  --preset fast       (1000ms = 1.0s, responsive)")
        print("  --preset very-fast  (500ms = 0.5s, quick)")
        print("\nOr set a custom time: --time 2500")
        
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure hw_bridge.py is running on port 7843")

if __name__ == "__main__":
    main()
