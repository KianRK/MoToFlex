import sys
sys.path.append('Simulator')

from motoflex_gym import WalkingSimulator
from motoflex_gym.gym_world import MoToFlexEnv

import gymnasium as gym
import numpy as np
from numpy.linalg import norm
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecVideoRecorder
import wandb
from wandb.integration.sb3 import WandbCallback

# For some more explanations, see envtest.ipynb.
obs_space = gym.spaces.Dict({
    "current_joint_angles": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "current_joint_velocities": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "current_body_orientation_quaternion": gym.spaces.Box(-1, 1, shape=(4,), dtype=float),
    "current_angular_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_lin_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "target_forwards_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_joint_torques": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "body_acceleration": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "p": gym.spaces.Box(-1, 1, shape=(2,), dtype=float)
})
 
obs_terms = lambda env, cycle_time, left_cycle_offset, right_cycle_offset: {
    "current_joint_angles": np.array(WalkingSimulator.get_joint_angles()),
    "current_joint_velocities": np.array(WalkingSimulator.get_joint_velocities()),
    "current_body_orientation_quaternion": np.array(WalkingSimulator.get_body_orientation_quaternion()),
    "current_angular_velocity": np.array(WalkingSimulator.get_angular_velocity()),
    "current_lin_vel": np.array(WalkingSimulator.get_velocity()),
    "target_forwards_vel": np.array([0.05, 0, 0]),
    "current_joint_torques": np.array(WalkingSimulator.get_joint_torques()),
    "body_acceleration": env.get_body_acceleration(),
    "p": np.array([np.sin((2*np.pi*((cycle_time+left_cycle_offset)%1)/50)), np.sin((2*np.pi*((cycle_time+right_cycle_offset)%1)/50))], dtype='float64')
    }

rew_terms = [
    lambda _, __, ___, ____: 15,
    lambda _, __, ___, periodic_reward_values: np.sum(WalkingSimulator.foot_contact(1) * periodic_reward_values["expected_c_frc_left"]),
    lambda _, __, ___, periodic_reward_values: np.sum(periodic_reward_values["expected_c_spd_left"] * norm(WalkingSimulator.get_left_foot_velocity())),
    lambda _, __, ___, periodic_reward_values: np.sum(WalkingSimulator.foot_contact(2) * periodic_reward_values["expected_c_frc_right"]),
    lambda _, __, ___, periodic_reward_values: np.sum(periodic_reward_values["expected_c_spd_right"] * norm(WalkingSimulator.get_right_foot_velocity())),
    lambda _, obs, __, ___: - 1 * np.sum(np.abs(20*(obs['target_forwards_vel'][0]-obs['current_lin_vel'][0]))),
    lambda env, obs, _, __: -1 * np.sum(env.compute_quaternion_difference(obs["current_body_orientation_quaternion"])),
    lambda _, __, last_action, ___: -0.01 * np.sum(np.abs(last_action)),
    lambda _, obs, __, ___: -1 * np.abs(obs["current_lin_vel"][1]),
    lambda _, obs, __, ___: -0.01 * np.sum(np.abs(obs["current_joint_torques"])),
    lambda _, obs, __, ___: -0.1 * np.sum(np.abs(obs["body_acceleration"])),
]
action_space = gym.spaces.Box( low=np.array([-0.38, -1.56, -0.09, -1.19, -0.4, -0.79, -1.56, -0.09, -1.19, -0.77]),
        high=np.array([0.79, 0.48, 2.11, 0.92, 0.77, 0.38, 0.48, 2.12, 0.93, 0.4]),
        dtype=float)
#action_space = gym.spaces.Box(-10, 10, shape=(10,), dtype=float)

action_function = lambda d: (d.tolist())

random_push = {
    "probability": 0.01,
    "force_range_x": [500, 1000]
}

def make_env():
    env = gym.make("MoToFlex/WalkingSimulator-v0", 
                render_mode='rgb_array', 
                observation_space = obs_space,
                observation_terms = obs_terms,
                reward_functions = rew_terms,
                action_space = action_space,
                action_function = action_function,
                ab_filter_alpha = 0.1,
                random_push = random_push

                )
    env = Monitor(env)  # record stats such as returns
    return env

if __name__ == "__main__":
    
    multi_input_lstm_policy_config = dict(lstm_hidden_size=128, n_lstm_layers=2, net_arch=[300, 300])

    recurrent_ppo_config = {
        "policy": "MultiInputLstmPolicy",
        "gae_lambda": 0.95,
        "gamma": 0.99,
        "n_steps": 1024,
        "batch_size": 32,
        "n_epochs": 4,
        "ent_coef": 0.01,
        "learning_rate": 0.0001,
        "clip_range": 0.2,
        "use_sde": True,
        "policy_kwargs": multi_input_lstm_policy_config,
        "sde_sample_freq": 4,
        "verbose": 1,
    }

    config = {
        "total_timesteps": 15e7
    }

    all_configs = {
        "learn_conf": config,
        "recurrent_ppo_config": recurrent_ppo_config,
        "reward_terms": rew_terms,
        "observation_space": obs_space,
        "observation_terms": obs_terms,
        "action_space": action_space,
        "action_function": action_function
    }
    
    run = wandb.init(
        name="recurrent-ppo",
        project="sb3",
        config=all_configs,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
        monitor_gym=True,  # auto-upload the videos of agents playing the game
        save_code=True,  # optional
    )

    if True:
        env = SubprocVecEnv([make_env for _ in range(20)])
    else:
        env = DummyVecEnv([make_env])

    env = VecVideoRecorder(
        env,
        f"tmp/videos/{run.id}",
        record_video_trigger=lambda x: x % 30000 == 0,
        video_length=200,
    )
    model = RecurrentPPO(
        env=env,
        **recurrent_ppo_config,
        tensorboard_log=f"tmp/runs/{run.id}"
        )

    model.learn(
        **config,
        callback=WandbCallback(
            gradient_save_freq=100,
            model_save_path=f"tmp/models/{run.id}",
            verbose=2,
        ),
    )
    run.finish()
