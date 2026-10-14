# Overhead Object Locator

This is a standalone vision framework. It does not import, call, or move the
IK model or physical arm. It detects changed objects inside a calibrated
tabletop workspace and emits JSON with `x_mm`, `y_mm`, and `z_mm`.

The camera should be rigidly mounted above the table. Aim it so the entire
pickup workspace in front of the arm is visible. Keep the arm outside the
workspace polygon whenever a background is captured.

## Coordinate convention

`configs/workspace.example.json` contains four physical workspace corners in
millimetres. Copy it to `configs/workspace.json`, then replace
`world_corners_mm` with the corners measured in the coordinate convention the
future IK model expects. The entries correspond to the image corners clicked
in this exact order: top-left, top-right, bottom-right, bottom-left.

For example, the provided points describe a 350 x 300 mm rectangle. They are
only an example: measure your own workspace from the arm base/origin before
using its output for motion.

`pick_z_mm` is the tabletop pickup plane. A regular monocular overhead camera
cannot measure an object's physical height accurately enough for reliable 3D
grasping. The framework reports this calibrated plane as Z, which is correct
for a table-surface pickup target. Add a depth camera or a calibrated
multi-view/known-object-height module later if Z must be the top surface.

## Setup

```powershell
cd CV-seg
pip install -r requirements-object-locator.txt
Copy-Item configs\workspace.example.json configs\workspace.json
```

## Calibrate and run

1. Put four visible marks at the real corners of the intended pickup area.
2. Edit `world_corners_mm` and `pick_z_mm` in `configs/workspace.json`.
3. Run the calibration tool and click those same four marks in order:

```powershell
python calibrate_workspace.py --config configs\workspace.json --camera 0
```

4. Remove objects from that workspace and capture the table background:

```powershell
python capture_background.py --config configs\workspace.json --output data\empty_workspace.png
```

The preview must show the intended camera feed before you press `c` to save.
Use `--camera 0`, `--camera 1`, and so on to select a different device. A
nearly black preview means that device index is not producing a usable feed.

5. Place an object in the workspace and start the locator:

```powershell
python locate_objects.py --config configs\workspace.json --background data\empty_workspace.png
```

The live overlay shows the calibrated polygon and detected centroids. stdout
contains records such as:

```json
{"objects": [{"id": 1, "x_mm": 264.8, "y_mm": -18.5, "z_mm": 0.0, "centroid_px": [821.2, 476.0], "area_px": 1533.5, "z_source": "workspace_pick_plane"}]}
```

Tune `detection.lab_threshold` upward if table texture or shadows are detected
as objects, or lower it if the item is missed. Tune `min_area_px` to ignore
small noise. Capture a new background whenever the camera moves, resolution
changes, or table lighting changes.

## Verification

Run the geometry and segmentation tests without any camera or arm connected:

```powershell
python -m unittest discover -s tests -v
```

## OpenCV window error on Windows

Calibration needs the normal `opencv-python` desktop package, not the
headless build. If `cv2.namedWindow` reports that the function is not
implemented, run:

```powershell
python -m pip uninstall -y opencv-python-headless
python -m pip install --force-reinstall opencv-python
```
