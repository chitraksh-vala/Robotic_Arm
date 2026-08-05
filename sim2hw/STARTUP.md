# SO-ARM101 Startup Guide

Start the three services in separate terminal windows:

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

**⚡ Motion Speed Control:** The hardware now uses smooth, controlled motion with Goal_Time=1500ms (1.5 seconds).
- To adjust: `python test_motion_speed.py --preset slow` (2s), `medium` (1.5s), or `fast` (1s)
- See MOTION_SPEED_README.md for details

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
