#!/usr/bin/env python3
"""Debug script for lerobot motor bus"""

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor, MotorNormMode, MotorCalibration
import sys

JOINT_NAMES = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
MOTOR_IDS  = [1, 2, 3, 4, 5, 6]

print("=" * 60)
print("LeRobot Feetech Motor Bus Debug")
print("=" * 60)

# Create motors
print("\n1. Creating motor objects...")
motors = {
    name: Motor(id=mid, model="sts3215", norm_mode=MotorNormMode.DEGREES)
    for name, mid in zip(JOINT_NAMES, MOTOR_IDS)
}
print(f"   Created {len(motors)} motor objects")

# Create calibration for each motor
print("\n2. Creating calibration data...")
calibration = {
    name: MotorCalibration(
        id=mid,
        drive_mode=0,  # position control
        homing_offset=2048,  # center position
        range_min=0,
        range_max=4095
    )
    for name, mid in zip(JOINT_NAMES, MOTOR_IDS)
}
print(f"   Created calibration for {len(calibration)} motors")

# Try to connect
print("\n3. Attempting connection...")
try:
    bus = FeetechMotorsBus(port="COM4", motors=motors, calibration=calibration)
    print("   FeetechMotorsBus created successfully")
    
    print("\n4. Calling connect()...")
    bus.connect()
    print("   Connected successfully!")
    
    print("\n5. Reading motor states...")
    try:
        states = bus.sync_read("Present_Position")
        print(f"   Present positions: {states}")
    except Exception as e:
        print(f"   Error reading: {e}")
    
    print("\n6. Testing motor write (small movement)...")
    test_cmd = {name: 10.0 for name in JOINT_NAMES}  # 10 degrees
    try:
        bus.sync_write("Goal_Position", test_cmd)
        print(f"   Sent command: {test_cmd}")
    except Exception as e:
        print(f"   Error writing: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n7. Disconnecting...")
    bus.disconnect()
    print("   Disconnected")
    
except Exception as e:
    print(f"   ERROR: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
