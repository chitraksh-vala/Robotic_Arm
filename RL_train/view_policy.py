"""
SO-101 Trained Policy Viewer
=============================
Watch your trained SAC policy run pick-and-place in the MuJoCo 3-D viewer.

Usage:
    python3 view_policy.py                                      # best model
    python3 view_policy.py --model checkpoints/sac_pickplace_final
    python3 view_policy.py --model checkpoints/best_model --episodes 20
    python3 view_policy.py --random                             # random policy baseline
    python3 view_policy.py --slow                               # slower playback

Controls (MuJoCo viewer):
    Left-drag   : orbit camera
    Right-drag  : pan
    Scroll      : zoom
    R           : reset camera
    Space       : pause/resume (not implemented — use --slow instead)

Output:
    Prints per-episode stats and a summary at the end.
"""

import argparse
import os
import sys
import time
import numpy as np

import mujoco
import mujoco.viewer

# ── Import env and training wrapper ──────────────────────────────────────────
sys.path.insert(0, os.path.dirname(__file__))
from so101_env import PickPlaceEnv, SO101BaseEnv
from train_pickplace import FlatObsWrapper

SCENE_PATH = os.path.join(os.path.dirname(__file__), "scene_full.xml")
DEFAULT_ACTION_SCALE = 0.04
DEFAULT_MAX_EPISODE_STEPS = 700


def run_viewer(
    model_path: str,
    n_episodes: int = 10,
    use_random: bool = False,
    step_delay: float = 0.02,   # seconds between steps (increase for slow-mo)
    seed: int = 0,
):
    """
    Run the trained policy (or random baseline) in the MuJoCo interactive viewer.
    """

    # ── Load model ────────────────────────────────────────────────────────────
    if not use_random:
        from stable_baselines3 import SAC
        if not os.path.exists(model_path) and not os.path.exists(model_path + ".zip"):
            print(f"ERROR: Model not found at {model_path}")
            print("Available checkpoints:")
            for f in os.listdir("checkpoints"):
                print(f"  checkpoints/{f}")
            sys.exit(1)
        print(f"Loading model: {model_path}")
        policy = SAC.load(model_path)
        print("Model loaded.\n")
    else:
        policy = None
        print("Running RANDOM policy (baseline).\n")

    # ── Create env with MuJoCo viewer ────────────────────────────────────────
    # We manage the viewer manually for more control
    mj_model = mujoco.MjModel.from_xml_path(
        SCENE_PATH
    )
    mj_data = mujoco.MjData(mj_model)

    # Create the RL env (headless — we sync to our own viewer)
    env = PickPlaceEnv(
        scene_path=SCENE_PATH,
        render_mode=None,
        action_scale=DEFAULT_ACTION_SCALE,
        max_episode_steps=DEFAULT_MAX_EPISODE_STEPS,
    )
    flat_env = FlatObsWrapper(env)

    print("Opening MuJoCo viewer window ...")
    print("  Tip: left-drag to orbit, scroll to zoom, right-drag to pan\n")

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:

        # Camera setup — good default view of the arm and table
        viewer.cam.azimuth  = 160
        viewer.cam.elevation = -20
        viewer.cam.distance  = 1.2
        viewer.cam.lookat[:] = [0.15, 0.0, 0.15]

        # ── Stats tracking ────────────────────────────────────────────────────
        all_successes = []
        all_returns   = []
        all_lengths   = []

        print(f"{'─'*60}")
        print(f"  {'Ep':>3}  {'Steps':>6}  {'Return':>9}  {'Success':>8}  {'Outcome'}")
        print(f"{'─'*60}")

        for ep in range(n_episodes):
            if not viewer.is_running():
                print("\nViewer closed — stopping.")
                break

            obs, _ = flat_env.reset(seed=seed + ep)

            # Sync viewer state with env's fresh reset
            _sync_viewer_state(mj_model, mj_data, env, viewer)

            total_r = 0.0
            steps   = 0

            while viewer.is_running():
                # ── Get action ────────────────────────────────────────────────
                if use_random:
                    action = flat_env.action_space.sample()
                else:
                    action, _ = policy.predict(obs, deterministic=True)

                # ── Step env ──────────────────────────────────────────────────
                obs, r, terminated, truncated, info = flat_env.step(action)
                total_r += r
                steps   += 1

                # ── Sync physics state → viewer ───────────────────────────────
                _sync_viewer_state(mj_model, mj_data, env, viewer)
                viewer.sync()

                # ── Pacing ────────────────────────────────────────────────────
                if step_delay > 0:
                    time.sleep(step_delay)

                if terminated or truncated:
                    break

            # ── Episode summary ───────────────────────────────────────────────
            outcome = "✅ SUCCESS" if info["success"] else (
                "❌ dropped" if info.get("obj_z", 0) < -0.02 else "⏱ timeout"
            )
            print(f"  {ep+1:>3}  {steps:>6}  {total_r:>9.2f}  "
                  f"{'YES' if info['success'] else 'no':>8}  {outcome}")

            all_successes.append(int(info["success"]))
            all_returns.append(total_r)
            all_lengths.append(steps)

            # Brief pause between episodes so you can see the reset
            if viewer.is_running():
                time.sleep(0.8)

        # ── Final summary ─────────────────────────────────────────────────────
        if all_successes:
            print(f"{'─'*60}")
            print(f"  Episodes     : {len(all_successes)}")
            print(f"  Success rate : {np.mean(all_successes)*100:.1f}%")
            print(f"  Mean return  : {np.mean(all_returns):.2f} ± {np.std(all_returns):.2f}")
            print(f"  Mean length  : {np.mean(all_lengths):.1f} steps")
            print(f"{'─'*60}")

    flat_env.close()


def _sync_viewer_state(mj_model, mj_data, env: PickPlaceEnv, viewer):
    """
    Copy the RL env's physics state into the viewer's mjData so the
    viewer always shows exactly what the policy sees.
    """
    mj_data.qpos[:] = env.data.qpos[:]
    mj_data.qvel[:] = env.data.qvel[:]
    mj_data.ctrl[:] = env.data.ctrl[:]
    mujoco.mj_forward(mj_model, mj_data)

    # Also update the goal disc visual position to match current goal
    goal_body_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "goal_visual")
    if goal_body_id >= 0:
        # Update the body position in model (cosmetic only)
        mj_model.body_pos[goal_body_id, 0] = env._goal_pos[0]
        mj_model.body_pos[goal_body_id, 1] = env._goal_pos[1]
        mj_model.body_pos[goal_body_id, 2] = 0.001


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="View trained SO-101 policy in MuJoCo")
    parser.add_argument("--model",    type=str, default="checkpoints/best_model",
                        help="Path to saved SAC model (default: checkpoints/best_model)")
    parser.add_argument("--episodes", type=int, default=10,
                        help="Number of episodes to run (default: 10)")
    parser.add_argument("--random",   action="store_true",
                        help="Run random policy instead (sanity check baseline)")
    parser.add_argument("--slow",     action="store_true",
                        help="Slow motion — 3x slower playback")
    parser.add_argument("--realtime", action="store_true",
                        help="Try to match real-time playback (20 Hz control)")
    parser.add_argument("--seed",     type=int, default=42)
    args = parser.parse_args()

    # Step delay:
    #   0       = as fast as possible
    #   0.02    = ~50 Hz, slightly faster than real-time
    #   0.05    = ~20 Hz, real-time
    #   0.15    = slow motion (good for inspection)
    if args.slow:
        delay = 0.15
    elif args.realtime:
        delay = 0.05
    else:
        delay = 0.02   # default: smooth but faster than real

    run_viewer(
        model_path=args.model,
        n_episodes=args.episodes,
        use_random=args.random,
        step_delay=delay,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
