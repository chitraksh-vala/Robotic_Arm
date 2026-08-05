# Robotic Arm Workspace

This repository collects the code and assets for a SO-ARM101 robotic arm setup across simulation, dual-arm control, reinforcement learning, and sim-to-hardware tooling.

## What’s inside

- `sim_train/` - MuJoCo simulation, controller input, recording, and training-data tooling.
- `dual_arm/` - Dual-arm MuJoCo scene, IK server, and browser dashboard for coordinated manipulation.
- `RL_train/` - RL environment, training script, policy viewer, and hardware bridge helpers.
- `sim2hw/` - Sim-to-hardware utilities, dashboards, and startup references.
- `CV-seg/` - Computer-vision segmentation helpers for item segregation workflows.


## Notes

- This repository currently organizes several related projects rather than a single app.
- Generated artifacts such as recordings, logs, checkpoints, caches, and build outputs should stay untracked.
- If you want, this root can later be split into a proper multi-package layout with shared dependencies and a unified launcher.