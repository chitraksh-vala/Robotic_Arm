"""
SO-101 Pick-and-Place RL Training
==================================
Uses Soft Actor-Critic (SAC) from Stable-Baselines3.

SAC is ideal here because:
  - Off-policy (sample efficient with replay buffer)
  - Handles continuous action spaces well
  - Entropy regularisation encourages exploration (critical for sparse rewards)

Run:
    python3 train_pickplace.py                     # train with defaults
    python3 train_pickplace.py --timesteps 1000000 # longer run
    python3 train_pickplace.py --render            # watch training (slow)
    python3 train_pickplace.py --eval_only checkpoints/best_model  # evaluate

TensorBoard:
    tensorboard --logdir logs/

Checkpoints are saved to checkpoints/ every 50k steps.
"""

import argparse
import os
import sys
import numpy as np
import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.callbacks import (
    CheckpointCallback, EvalCallback, CallbackList
)
from stable_baselines3.common.monitor import Monitor

# Import our env (from same directory)
sys.path.insert(0, os.path.dirname(__file__))
from so101_env import PickPlaceEnv, SO101BaseEnv

SCENE_PATH = os.path.join(os.path.dirname(__file__), "scene_full.xml")
REPLAY_BUFFER_PATH = os.path.join("checkpoints", "sac_pickplace_replay_buffer.pkl")
FINAL_REPLAY_BUFFER_PATH = os.path.join("checkpoints", "sac_pickplace_final_replay_buffer.pkl")
DEFAULT_ACTION_SCALE = 0.04
DEFAULT_MAX_EPISODE_STEPS = 700


class ReplayBufferCallback(BaseCallback):
    def __init__(self, save_freq: int, save_path: str, verbose: int = 0):
        super().__init__(verbose)
        self.save_freq = max(int(save_freq), 1)
        self.save_path = save_path

    def _on_step(self) -> bool:
        if self.num_timesteps > 0 and self.num_timesteps % self.save_freq == 0:
            self.model.save_replay_buffer(self.save_path)
            if self.verbose > 0:
                print(f"Saved replay buffer -> {self.save_path}")
        return True

# ── Flat-obs wrapper ──────────────────────────────────────────────────────────
class FlatObsWrapper(gym.ObservationWrapper):
    """
    Wraps the Dict observation into a flat Box vector.
    SB3's SAC/PPO work with Box obs natively; Dict requires extra setup.
    Shape: (36,) — see SO101BaseEnv.flat_obs_dim
    """
    def __init__(self, env: gym.Env):
        super().__init__(env)
        dim = env.flat_obs_dim
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(dim,), dtype=np.float32
        )

    def observation(self, obs):
        return SO101BaseEnv.flatten_obs(obs)


def make_env(rank: int = 0, seed: int = 0, render: bool = False):
    """Factory function for vectorised env creation."""
    def _init():
        render_mode = "human" if (render and rank == 0) else None
        env = PickPlaceEnv(
            scene_path=SCENE_PATH,
            render_mode=render_mode,
            action_scale=DEFAULT_ACTION_SCALE,
            sim_substeps=10,
            max_episode_steps=DEFAULT_MAX_EPISODE_STEPS,
        )
        env = FlatObsWrapper(env)
        env = Monitor(env)
        env.reset(seed=seed + rank)
        return env
    return _init


# ── Evaluation helper ─────────────────────────────────────────────────────────
def evaluate(model, n_episodes: int = 20, render: bool = False):
    env = PickPlaceEnv(
        scene_path=SCENE_PATH,
        render_mode="human" if render else None,
        action_scale=DEFAULT_ACTION_SCALE,
        max_episode_steps=DEFAULT_MAX_EPISODE_STEPS,
    )
    env = FlatObsWrapper(env)

    successes, returns, lengths = [], [], []
    for ep in range(n_episodes):
        obs, _ = env.reset(seed=ep * 7)
        total_r, steps = 0.0, 0
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, terminated, truncated, info = env.step(action)
            total_r += r
            steps   += 1
            if terminated or truncated:
                break
        successes.append(int(info["success"]))
        returns.append(total_r)
        lengths.append(steps)

    env.close()
    print(f"\n{'─'*50}")
    print(f"  Eval over {n_episodes} episodes:")
    print(f"  Success rate : {np.mean(successes)*100:.1f}%")
    print(f"  Mean return  : {np.mean(returns):.2f} ± {np.std(returns):.2f}")
    print(f"  Mean length  : {np.mean(lengths):.1f} steps")
    print(f"{'─'*50}\n")
    return np.mean(successes)


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps",  type=int,   default=500_000,
                        help="Total training timesteps")
    parser.add_argument("--n_envs",     type=int,   default=4,
                        help="Parallel environments (use 1 if debugging)")
    parser.add_argument("--render",     action="store_true",
                        help="Render the first env during training (slow)")
    parser.add_argument("--eval_only",  type=str,   default=None,
                        help="Path to saved model — skip training, just evaluate")
    parser.add_argument("--checkpoint", type=str,   default=None,
                        help="Resume training from checkpoint path")
    parser.add_argument("--seed",       type=int,   default=42)
    args = parser.parse_args()

    os.makedirs("checkpoints", exist_ok=True)
    os.makedirs("logs",        exist_ok=True)

    # ── Eval-only mode ────────────────────────────────────────────────────────
    if args.eval_only:
        print(f"Loading model from {args.eval_only} ...")
        model = SAC.load(args.eval_only)
        evaluate(model, n_episodes=20, render=args.render)
        return

    # ── Build vectorised training env ─────────────────────────────────────────
    print(f"Launching {args.n_envs} parallel envs ...")
    # DummyVecEnv for <= 2 envs (less overhead), SubprocVecEnv for more
    VecEnvCls = DummyVecEnv if args.n_envs <= 2 else SubprocVecEnv
    vec_env = make_vec_env(
        make_env(rank=0, seed=args.seed, render=args.render),
        n_envs=args.n_envs,
        vec_env_cls=VecEnvCls,
        env_kwargs=None,
        # Use the factory for each rank
    )
    # Rebuild with per-rank seeds
    env_fns = [make_env(rank=i, seed=args.seed, render=(args.render and i==0))
               for i in range(args.n_envs)]
    vec_env = VecEnvCls(env_fns)

    # ── Eval env (single, deterministic) ─────────────────────────────────────
    eval_env = DummyVecEnv([make_env(rank=0, seed=args.seed + 999)])

    # ── Callbacks ─────────────────────────────────────────────────────────────
    checkpoint_cb = CheckpointCallback(
        save_freq=max(50_000 // args.n_envs, 1),
        save_path="checkpoints/",
        name_prefix="sac_pickplace",
        verbose=1,
    )

    replay_cb = ReplayBufferCallback(
        save_freq=max(50_000, 1),
        save_path=REPLAY_BUFFER_PATH,
        verbose=1,
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path="checkpoints/",
        log_path="logs/eval/",
        eval_freq=max(25_000 // args.n_envs, 1),
        n_eval_episodes=10,
        deterministic=True,
        render=False,
        verbose=1,
    )

    callbacks = CallbackList([checkpoint_cb, replay_cb, eval_cb])

    # ── SAC hyperparameters ───────────────────────────────────────────────────
    # Tuned for this task:
    #   - Large replay buffer (off-policy, sample efficiency)
    #   - High learning starts to fill buffer before first update
    #   - ent_coef="auto" lets SAC tune exploration automatically
    sac_kwargs = dict(
        policy            = "MlpPolicy",
        env               = vec_env,
        learning_rate     = 2e-4,
        buffer_size       = 500_000,
        learning_starts   = 5000,
        batch_size        = 512,
        tau               = 0.005,
        gamma             = 0.99,
        train_freq        = (1, "step"),
        gradient_steps    = 2,
        ent_coef          = "auto",
        target_entropy    = "auto",
        policy_kwargs     = dict(net_arch=[256, 256, 256], use_sde=False),
        tensorboard_log   = "logs/",
        verbose           = 1,
        seed              = args.seed,
    )

    if args.checkpoint:
        print(f"Resuming from checkpoint: {args.checkpoint}")
        model = SAC.load(args.checkpoint, env=vec_env, **{
            k: v for k, v in sac_kwargs.items()
            if k not in ("policy", "env")
        })
        replay_buffer_path = os.path.splitext(args.checkpoint)[0] + "_replay_buffer.pkl"
        if os.path.exists(replay_buffer_path):
            print(f"Loading replay buffer from {replay_buffer_path}")
            model.load_replay_buffer(replay_buffer_path)
        else:
            print("No matching replay buffer found; resuming model weights only.")
    else:
        model = SAC(**sac_kwargs)

    # ── Train ─────────────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"  SO-101 Pick-and-Place  —  SAC Training")
    print(f"  Timesteps : {args.timesteps:,}")
    print(f"  Envs      : {args.n_envs}")
    print(f"  Obs dim   : 36  |  Act dim : 6")
    print(f"{'='*60}\n")

    model.learn(
        total_timesteps       = args.timesteps,
        callback              = callbacks,
        reset_num_timesteps   = not bool(args.checkpoint),
        progress_bar          = True,
    )

    # ── Save final model ──────────────────────────────────────────────────────
    model.save("checkpoints/sac_pickplace_final")
    model.save_replay_buffer(FINAL_REPLAY_BUFFER_PATH)
    print("\nSaved final model → checkpoints/sac_pickplace_final.zip")
    print(f"Saved final replay buffer → {FINAL_REPLAY_BUFFER_PATH}")

    # ── Final evaluation ──────────────────────────────────────────────────────
    evaluate(model, n_episodes=20, render=args.render)

    vec_env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
