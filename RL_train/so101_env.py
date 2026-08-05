"""
SO-101 Gymnasium Environment
============================
A reusable RL environment for the SO-101 robot arm built on MuJoCo 3.

Structure
---------
SO101BaseEnv   — robot loading, obs/act spaces, physics step, rendering
PickPlaceEnv   — pick-and-place task: reward, reset, success criteria

Extending to new tasks
----------------------
Subclass SO101BaseEnv and implement:
  _task_obs()       → extra obs dict entries (object poses, goal, etc.)
  _compute_reward() → (float, info_dict)
  _is_success()     → bool
  _is_failure()     → bool  (optional early termination)
  _reset_task()     → place objects / set goal each episode

Observation (Dict)
------------------
  qpos          (6,)  — arm+gripper joint positions, normalized [-1,1]
  qvel          (6,)  — joint velocities
  ee_pos        (3,)  — end-effector XYZ world frame
  ee_quat       (4,)  — end-effector orientation quaternion
  obj_pos       (3,)  — object XYZ
  obj_quat      (4,)  — object orientation
  goal_pos      (3,)  — target drop zone XYZ
  rel_obj_ee    (3,)  — object pos relative to EE  (approach cue)
  rel_obj_goal  (3,)  — object pos relative to goal (placement cue)
  gripper       (1,)  — gripper opening [0=closed, 1=open]

Action space: Box(6,) joint-position deltas, normalized [-1,1]
             scaled by action_scale × per-joint range half-width
"""

from __future__ import annotations

import os, math
import numpy as np
from typing import Any, Dict, Optional, Tuple

import mujoco
import gymnasium as gym
from gymnasium import spaces

# ── Scene path ───────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
_RL_SCENE   = os.path.join(_HERE, "scene_full.xml")

# ── Joint layout ─────────────────────────────────────────────────────────────
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll"]
ALL_ACTS   = ARM_JOINTS + ["gripper"]
EE_SITE    = "gripperframe"

# Control ranges (matches actuator ctrlrange in XML)
_CTRL_LO    = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.17453])
_CTRL_HI    = np.array([ 1.91986,  1.74533,  1.69,  1.65806,  2.84121,  1.74533])
_CTRL_MID   = (_CTRL_LO + _CTRL_HI) / 2.0
_CTRL_RANGE = (_CTRL_HI - _CTRL_LO) / 2.0


# ══════════════════════════════════════════════════════════════════════════════
#  Base environment
# ══════════════════════════════════════════════════════════════════════════════
class SO101BaseEnv(gym.Env):
    """
    Handles physics, spaces, and rendering.
    Subclass and override the four task methods.
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 20}

    def __init__(
        self,
        scene_path: str = _RL_SCENE,
        render_mode: Optional[str] = None,
        action_scale: float = 0.04,    # fraction of joint range per step
        control_hz: int = 20,
        sim_substeps: int = 10,        # physics steps per control step
        max_episode_steps: int = 700,
        **kwargs,
    ):
        super().__init__()
        self.render_mode      = render_mode
        self.action_scale     = action_scale
        self.control_hz       = control_hz
        self.sim_substeps     = sim_substeps
        self.max_episode_steps = max_episode_steps

        # ── Load model ───────────────────────────────────────────────────────
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data  = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        # ── Cache ids ────────────────────────────────────────────────────────
        self._ee_id   = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
        self._act_ids = {n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                         for n in ALL_ACTS}
        self._jnt_ids = {n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                         for n in ALL_ACTS}

        # ── Spaces ───────────────────────────────────────────────────────────
        self.action_space = spaces.Box(-1.0, 1.0, shape=(6,), dtype=np.float32)

        self.observation_space = spaces.Dict({
            "qpos":         spaces.Box(-1.0,   1.0,   (6,), np.float32),
            "qvel":         spaces.Box(-np.inf, np.inf,(6,), np.float32),
            "ee_pos":       spaces.Box(-2.0,   2.0,   (3,), np.float32),
            "ee_quat":      spaces.Box(-1.0,   1.0,   (4,), np.float32),
            "obj_pos":      spaces.Box(-2.0,   2.0,   (3,), np.float32),
            "obj_quat":     spaces.Box(-1.0,   1.0,   (4,), np.float32),
            "goal_pos":     spaces.Box(-2.0,   2.0,   (3,), np.float32),
            "rel_obj_ee":   spaces.Box(-2.0,   2.0,   (3,), np.float32),
            "rel_obj_goal": spaces.Box(-2.0,   2.0,   (3,), np.float32),
            "gripper":      spaces.Box( 0.0,   1.0,   (1,), np.float32),
        })

        # ── Renderer ─────────────────────────────────────────────────────────
        self._renderer: Optional[mujoco.Renderer] = None
        if render_mode == "human":
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self._viewer = None

        self._step_count    = 0
        self._episode_count = 0

    # ── Physics helpers ───────────────────────────────────────────────────────
    def _get_ctrl(self) -> np.ndarray:
        return np.array([self.data.ctrl[self._act_ids[n]] for n in ALL_ACTS])

    def _set_ctrl(self, ctrl: np.ndarray):
        for i, n in enumerate(ALL_ACTS):
            self.data.ctrl[self._act_ids[n]] = float(ctrl[i])

    def _get_qpos(self) -> np.ndarray:
        q = np.array([
            self.data.qpos[self.model.joint(self._jnt_ids[n]).qposadr[0]]
            for n in ALL_ACTS
        ])
        return ((q - _CTRL_MID) / _CTRL_RANGE).astype(np.float32)

    def _get_qvel(self) -> np.ndarray:
        return np.array([
            self.data.qvel[self.model.joint(self._jnt_ids[n]).dofadr[0]]
            for n in ALL_ACTS
        ], dtype=np.float32)

    def _get_ee_pos(self) -> np.ndarray:
        return self.data.site_xpos[self._ee_id].copy().astype(np.float32)

    def _get_ee_quat(self) -> np.ndarray:
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, self.data.site_xmat[self._ee_id])
        return q.astype(np.float32)

    def _get_gripper_opening(self) -> float:
        g = self.data.qpos[self.model.joint(self._jnt_ids["gripper"]).qposadr[0]]
        return float(np.clip((g + 0.17453) / (1.74533 + 0.17453), 0.0, 1.0))

    def _step_physics(self):
        for _ in range(self.sim_substeps):
            mujoco.mj_step(self.model, self.data)

    # ── Task interface ────────────────────────────────────────────────────────
    def _task_obs(self) -> Dict[str, np.ndarray]:
        raise NotImplementedError

    def _compute_reward(self, obs: Dict, action: np.ndarray) -> Tuple[float, Dict]:
        raise NotImplementedError

    def _is_success(self, obs: Dict) -> bool:
        raise NotImplementedError

    def _is_failure(self, obs: Dict) -> bool:
        return False

    def _reset_task(self, rng: np.random.Generator):
        raise NotImplementedError

    # ── Gym API ──────────────────────────────────────────────────────────────
    def _build_obs(self) -> Dict[str, np.ndarray]:
        task    = self._task_obs()
        ee_pos  = self._get_ee_pos()
        obj_pos = task.get("obj_pos",  np.zeros(3, dtype=np.float32))
        goal_pos = task.get("goal_pos", np.zeros(3, dtype=np.float32))
        return {
            "qpos":         self._get_qpos(),
            "qvel":         self._get_qvel(),
            "ee_pos":       ee_pos,
            "ee_quat":      self._get_ee_quat(),
            "obj_pos":      obj_pos,
            "obj_quat":     task.get("obj_quat", np.array([1,0,0,0], dtype=np.float32)),
            "goal_pos":     goal_pos,
            "rel_obj_ee":   (obj_pos - ee_pos).astype(np.float32),
            "rel_obj_goal": (obj_pos - goal_pos).astype(np.float32),
            "gripper":      np.array([self._get_gripper_opening()], dtype=np.float32),
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = np.random.default_rng(seed)

        mujoco.mj_resetData(self.model, self.data)
        self._set_ctrl(np.zeros(6))
        mujoco.mj_forward(self.model, self.data)

        self._reset_task(rng)

        # Brief settle
        for _ in range(50):
            mujoco.mj_step(self.model, self.data)

        self._step_count = 0
        self._episode_count += 1

        obs = self._build_obs()
        return obs, {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float64)

        current = self._get_ctrl()
        delta   = action * self.action_scale * _CTRL_RANGE
        new_ctrl = np.clip(current + delta, _CTRL_LO, _CTRL_HI)
        self._set_ctrl(new_ctrl)

        self._step_physics()
        self._step_count += 1

        obs              = self._build_obs()
        reward, info     = self._compute_reward(obs, action)
        success          = self._is_success(obs)
        failure          = self._is_failure(obs)
        timeout          = self._step_count >= self.max_episode_steps

        terminated = success or failure
        truncated  = timeout

        info["success"]      = success
        info["episode_step"] = self._step_count

        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, height=480, width=640)
            self._renderer.update_scene(self.data)
            return self._renderer.render()
        elif self.render_mode == "human" and self._viewer is not None:
            if self._viewer.is_running():
                self._viewer.sync()

    def close(self):
        if self._viewer is not None:
            try: self._viewer.close()
            except: pass
        if self._renderer is not None:
            self._renderer.close()

    @staticmethod
    def flatten_obs(obs: Dict[str, np.ndarray]) -> np.ndarray:
        """Flatten Dict obs to a 1-D vector (for non-Dict RL algorithms)."""
        return np.concatenate([obs[k].flatten() for k in (
            "qpos", "qvel", "ee_pos", "ee_quat",
            "obj_pos", "obj_quat", "goal_pos",
            "rel_obj_ee", "rel_obj_goal", "gripper",
        )]).astype(np.float32)

    @property
    def flat_obs_dim(self) -> int:
        return 6+6+3+4+3+4+3+3+3+1  # = 36


# ══════════════════════════════════════════════════════════════════════════════
#  Pick-and-Place Task
# ══════════════════════════════════════════════════════════════════════════════
class PickPlaceEnv(SO101BaseEnv):
    """
    Task: pick the cylinder from a randomized spawn → place at randomized goal.

    Reward (dense + sparse):
      reach_bonus   −‖EE − obj‖              pulls EE toward object
      grasp_bonus   +W_GRASP/step             when gripper closed around object
      lift_bonus    +W_LIFT/step              while object is above lift_height
      place_bonus   −W_PLACE·‖obj_xy−goal_xy‖ while lifted, pulls toward goal
      success_bonus +W_SUCCESS (sparse)        episode ends on success
      drop_penalty  W_DROP                    if object falls off table

    Domain randomisation (per episode):
      Object XY: uniform in SPAWN_REGION
      Goal   XY: uniform in GOAL_REGION, at least MIN_DIST from spawn
    """

    # Regions (x_min, x_max, y_min, y_max)
    SPAWN_REGION = (0.18, 0.30, -0.12, 0.12)
    GOAL_REGION  = (0.12, 0.25, -0.18, 0.18)
    MIN_OBJ_GOAL_DIST = 0.10

    OBJ_HALF_H   = 0.02      # cylinder half-height (from XML size="0.015 0.02")
    LIFT_HEIGHT  = 0.06      # z threshold to count as "lifted"
    SUCCESS_DIST = 0.03      # XY radius for success
    SUCCESS_Z_MAX= 0.05      # object must be near table height at goal

    W_REACH   = 2.0    # was 1.0 — stronger pull toward object
    W_GRASP   = 3.0    # was 1.0 — bigger incentive to actually close gripper
    W_LIFT    = 1.0    # was 0.5 — reward lifting more
    W_PLACE   = 3.0    # was 2.0
    W_SUCCESS = 150.0  # was 100.0
    W_DROP    = -5.0
    GRASP_DIST_THRESH = 0.05
    GRIPPER_CLOSE_THRESH = 0.25

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Body id for pick_object
        self._obj_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "pick_object"
        )
        # Goal visual body
        self._goal_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "goal_visual"
        )

        # Find freejoint attached to pick_object
        self._obj_jnt_qposadr = None
        self._obj_jnt_dofadr  = None
        for j in range(self.model.njnt):
            if (self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE and
                    self.model.jnt_bodyid[j] == self._obj_body_id):
                self._obj_jnt_qposadr = int(self.model.jnt_qposadr[j])
                self._obj_jnt_dofadr  = int(self.model.jnt_dofadr[j])
                break
        assert self._obj_jnt_qposadr is not None, "pick_object freejoint not found"

        self._goal_pos = np.array([0.18, 0.0, 0.0], dtype=np.float32)

    # ── Object helpers ────────────────────────────────────────────────────────
    def _get_obj_pos(self) -> np.ndarray:
        a = self._obj_jnt_qposadr
        return self.data.qpos[a:a+3].copy().astype(np.float32)

    def _get_obj_quat(self) -> np.ndarray:
        a = self._obj_jnt_qposadr
        return self.data.qpos[a+3:a+7].copy().astype(np.float32)

    def _set_obj_pose(self, pos: np.ndarray):
        a = self._obj_jnt_qposadr
        self.data.qpos[a:a+3] = pos
        self.data.qpos[a+3]   = 1.0   # qw
        self.data.qpos[a+4:a+7] = 0.0
        da = self._obj_jnt_dofadr
        self.data.qvel[da:da+6] = 0.0

    def _move_goal_visual(self, goal_xy: np.ndarray):
        """Slide the goal disc geom to match the current goal position."""
        # goal_visual body pos is baked at load time; we update via mocap or
        # just let the visual be approximate — update qpos if it's a freejoint,
        # otherwise we reposition it at reset via model.body_pos (not data).
        # Simplest: stash in self._goal_pos, render as overlay if needed.
        pass  # visual disc is cosmetic; policy uses self._goal_pos directly

    # ── Task interface ────────────────────────────────────────────────────────
    def _task_obs(self) -> Dict[str, np.ndarray]:
        return {
            "obj_pos":  self._get_obj_pos(),
            "obj_quat": self._get_obj_quat(),
            "goal_pos": self._goal_pos.copy(),
        }

    def _reset_task(self, rng: np.random.Generator):
        sx0, sx1, sy0, sy1 = self.SPAWN_REGION
        gx0, gx1, gy0, gy1 = self.GOAL_REGION

        for _ in range(200):
            ox = rng.uniform(sx0, sx1)
            oy = rng.uniform(sy0, sy1)
            gx = rng.uniform(gx0, gx1)
            gy = rng.uniform(gy0, gy1)
            if math.hypot(ox - gx, oy - gy) >= self.MIN_OBJ_GOAL_DIST:
                break

        self._set_obj_pose(np.array([ox, oy, self.OBJ_HALF_H]))
        self._goal_pos = np.array([gx, gy, 0.0], dtype=np.float32)
        mujoco.mj_forward(self.model, self.data)

    def _compute_reward(self, obs: Dict, action: np.ndarray) -> Tuple[float, Dict]:
        ee_pos   = obs["ee_pos"]
        obj_pos  = obs["obj_pos"]
        goal_pos = obs["goal_pos"]
        gripper  = float(obs["gripper"][0])
        obj_z    = float(obj_pos[2])

        d_reach = float(np.linalg.norm(ee_pos - obj_pos))
        r_reach = -d_reach * self.W_REACH

        lift_progress = float(np.clip(obj_z / self.LIFT_HEIGHT, 0.0, 1.0))
        lifted   = lift_progress >= 1.0
        r_lift   = self.W_LIFT * lift_progress

        grasp_dist = float(np.clip((self.GRASP_DIST_THRESH - d_reach) / self.GRASP_DIST_THRESH, 0.0, 1.0))
        grasp_close = float(np.clip((self.GRIPPER_CLOSE_THRESH - gripper) / self.GRIPPER_CLOSE_THRESH, 0.0, 1.0))
        grasping = (d_reach < self.GRASP_DIST_THRESH) and (gripper < self.GRIPPER_CLOSE_THRESH)
        r_grasp  = self.W_GRASP * grasp_dist * grasp_close

        r_place  = 0.0
        d_place  = float(np.linalg.norm(obj_pos - goal_pos))
        if lifted:
            r_place = -d_place * self.W_PLACE

        r_drop   = self.W_DROP if obj_z < -0.02 else 0.0
        r_success= self.W_SUCCESS if self._is_success(obs) else 0.0

        reward = r_reach + r_lift + r_grasp + r_place + r_drop + r_success

        info = dict(
            r_reach=r_reach, r_lift=r_lift, r_grasp=r_grasp,
            r_place=r_place, r_drop=r_drop, r_success=r_success,
            d_reach=d_reach, d_place=d_place,
            obj_z=obj_z, lifted=lifted, grasping=grasping,
        )
        return float(reward), info

    def _is_success(self, obs: Dict) -> bool:
        obj_pos  = obs["obj_pos"]
        goal_pos = obs["goal_pos"]
        xy_dist = float(np.linalg.norm(obj_pos[:2] - goal_pos[:2]))
        return (
            xy_dist < self.SUCCESS_DIST
            and float(obj_pos[2]) > self.LIFT_HEIGHT * 0.5
            and float(obj_pos[2]) < self.SUCCESS_Z_MAX
        )

    def _is_failure(self, obs: Dict) -> bool:
        return float(obs["obj_pos"][2]) < -0.05


# ══════════════════════════════════════════════════════════════════════════════
#  Gym registration
# ══════════════════════════════════════════════════════════════════════════════
gym.register(
    id="SO101PickPlace-v0",
    entry_point=f"{__name__}:PickPlaceEnv",
    max_episode_steps=500,
)


# ══════════════════════════════════════════════════════════════════════════════
#  Sanity check
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import time

    print("Creating PickPlaceEnv ...")
    env = PickPlaceEnv(render_mode=None, max_episode_steps=200)

    print(f"Action space   : {env.action_space}")
    print(f"Obs keys       : {list(env.observation_space.keys())}")
    print(f"Flat obs dim   : {env.flat_obs_dim}")

    print("\nRunning 3 random episodes ...")
    for ep in range(3):
        obs, _ = env.reset(seed=ep)
        total_r, steps = 0.0, 0
        t0 = time.perf_counter()
        while True:
            action = env.action_space.sample()
            obs, r, terminated, truncated, info = env.step(action)
            total_r += r
            steps   += 1
            if terminated or truncated:
                break
        fps = steps / (time.perf_counter() - t0)
        print(f"  Ep {ep+1}: {steps:3d} steps | Σr={total_r:7.2f} | "
              f"success={info['success']} | {fps:.0f} steps/s | "
              f"obj_z={info['obj_z']:.3f}")

    env.close()
    print("\nDone. Import PickPlaceEnv from so101_env.py to start training.")
