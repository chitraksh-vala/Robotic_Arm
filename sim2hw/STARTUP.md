# SO-ARM101 Startup Guide

**Shortcut:** from the repository root, `.\run.ps1 -Port COM4 -Detector none -Dashboard`
starts all three services below in their own windows (run `.\setup.ps1` once
first). Without `-Detector none` it also starts the camera object detector.
Full reference: [README.md](README.md).

Or start the three services by hand in separate terminal windows:

## 1. IK Server (port 7842)
```powershell
python ik_server.py --viewer
```
Or headless (no interactive viewer but can be initiated later from the dashboard):
```powershell
python ik_server.py
```

## 2. Hardware Bridge (port 7843)
```powershell
lerobot-find-port
python hw_bridge.py --port COM4 
```
*Replace `COM4` with your actual serial port found from lerobot-find-port (e.g., `/dev/ttyUSB0` on Linux)*

**Motion:** the bridge streams each motion to the servos at 50 Hz and applies
LeRobot's SO-101 servo gains on connect, so moves are smooth. Each move's
duration comes from the IK server (the object detector uses 4 s; change it with
`--move-duration`). To set the duration of manual `/move` commands:
`python test_motion_speed.py --preset slow` (2 s), `medium` (1.5 s) or `fast` (1 s).
Servo tuning and gripper options (`--p-gain`, `--grip-squeeze-deg`,
`--log-health` …) are listed in [README.md](README.md#hardware-bridge-hw_bridgepy).

## 3. Dashboard HTTP Server (port 8000)
Required to bypass CORS protocol issues between dashboard and hardware bridge
```powershell
python -m http.server 8000
```

## Access the Dashboard
Once all three services are running, open your browser to:
```
http://127.0.0.1:8000/so101_dashboard.html
```

---

### Service Endpoints
- **IK Server**: `http://127.0.0.1:7842`
- **Hardware Bridge**: `http://127.0.0.1:7843  ` (optional)
- **Dashboard**: `http://127.0.0.1:8000/so101_dashboard.html`
