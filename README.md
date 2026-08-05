# Robotic Arm Workspace

This repository collects the code and assets for a SO-ARM101 robotic arm setup across simulation, dual-arm control, reinforcement learning, and sim-to-hardware tooling.

## What’s inside

- `sim_train/` - MuJoCo simulation, controller input, recording, and training-data tooling.
- `dual_arm/` - Dual-arm MuJoCo scene, IK server, and browser dashboard for coordinated manipulation.
- `RL_train/` - RL environment, training script, policy viewer, and hardware bridge helpers.
- `sim2hw/` - Sim-to-hardware utilities, dashboards, and startup references.
- `CV-seg/` - Computer-vision segmentation helpers for item segregation workflows.

## Good starting points

If you want to run the simulator first, start with `sim_train/README.md`.

If you want the dual-arm stack, start with `dual_arm/DUAL_STARTUP.md`.

If you want RL training, inspect `RL_train/so101_env.py` and `RL_train/train_pickplace.py`.

If you want to connect simulation to hardware, start with `sim2hw/STARTUP.md` and `sim2hw/QUICK_REFERENCE.md`.

## Suggested workflow

1. Install the Python dependencies for the subproject you want to work on.
2. Validate the MuJoCo model or dashboard entry point.
3. Run the relevant server, simulator, or training script.
4. Save logs, checkpoints, and recordings outside version control.

## Notes

- This repository currently organizes several related projects rather than a single app.
- Generated artifacts such as recordings, logs, checkpoints, caches, and build outputs should stay untracked.
- If you want, this root can later be split into a proper multi-package layout with shared dependencies and a unified launcher.