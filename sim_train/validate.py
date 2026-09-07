"""
Validation Script
Verify that all components are properly installed and configured
"""

import sys
from pathlib import Path

print("=" * 60)
print("SO Arm Simulator - Installation Validator")
print("=" * 60)
print()

# Track validation results
checks_passed = 0
checks_failed = 0

def check(name: str, condition: bool, help_text: str = ""):
    """Run a validation check"""
    global checks_passed, checks_failed
    
    status = "✓ PASS" if condition else "✗ FAIL"
    print(f"  {status}: {name}")
    
    if not condition and help_text:
        print(f"         → {help_text}")
    
    if condition:
        checks_passed += 1
    else:
        checks_failed += 1


print("[1] Checking Python Environment")
print("-" * 60)

# Python version
import sys
py_version = sys.version_info
check("Python version", py_version >= (3, 8), 
      f"Current: {py_version.major}.{py_version.minor}. Required: 3.8+")

# Required modules
print()
print("[2] Checking Python Dependencies")
print("-" * 60)

required_modules = {
    'mujoco': ('MuJoCo physics engine', 'mujoco'),
    'numpy': ('Numerical computing', 'numpy'),
    'PIL': ('PNG image rendering via Pillow', 'pillow'),
    'h5py': ('HDF5 file format', 'h5py'),
    'yaml': ('Configuration files', 'pyyaml'),
}

for module_name, (description, package_name) in required_modules.items():
    try:
        __import__(module_name)
        check(f"{module_name}", True)
    except ImportError as e:
        check(f"{module_name}", False, 
              f"Install with: pip install {package_name}")

# Project structure
print()
print("[3] Checking Project Structure")
print("-" * 60)

base_path = Path(__file__).parent
required_dirs = {
    'mujoco_model': 'MuJoCo model definitions',
    'src': 'Python source code',
    'recordings': 'Motion recordings',
    'config': 'Configuration files',
}

for dir_name, description in required_dirs.items():
    dir_path = base_path / dir_name
    check(f"{dir_name}/ directory", dir_path.exists(), 
          f"Create with: mkdir {dir_name}")

required_files = {
    'mujoco_model/so_arm.xml': 'Arm model definition',
    'src/simulator.py': 'Main simulator',
    'src/controller.py': 'Controller interface',
    'src/recorder.py': 'Recording system',
    'requirements.txt': 'Dependencies list',
}

for file_name, description in required_files.items():
    file_path = base_path / file_name
    check(f"{file_name}", file_path.exists(), 
          f"Missing: {description}")

# Summary
print()
print("=" * 60)
print(f"Validation Summary: {checks_passed} passed, {checks_failed} failed")
print("=" * 60)

if checks_failed == 0:
    print()
    print("✓ All checks passed! Ready to run simulator.")
    print()
    print("Quick start:")
    print("  cd src")
    print("  python simulator.py")
    sys.exit(0)
else:
    print()
    print("✗ Some checks failed. Please fix the issues above.")
    print()
    print("Common fixes:")
    print("  1. Install dependencies: pip install -r requirements.txt")
    print("  2. Create missing directories: mkdir mujoco_model recordings config")
    sys.exit(1)
