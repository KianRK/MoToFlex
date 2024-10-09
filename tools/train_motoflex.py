import sys
sys.path.append('Simulator')

from motoflex_gym import WalkingSimulator
from motoflex_gym.gym_world import MoToFlexEnv

import gymnasium as gym
import numpy as np
from numpy.linalg import norm
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecVideoRecorder, VecNormalize
import wandb
from wandb.integration.sb3 import WandbCallback

# For some more explanations, see envtest.ipynb.

upper_joint_limits = [0.379, 1.53, 0.09, 1.189, 0.397, 0.79, 1.53, 0.09, 1.18, 0.76]
lower_joint_limits = [-0.79, -0.48, -2.11, -0.92, -0.76, -0.37, -0.48, -2.12, -0.93, -0.397]


obs_space = gym.spaces.Dict({
    "left_foot_contact": gym.spaces.Discrete(2),
    "right_foot_contact": gym.spaces.Discrete(2),
    "left_foot_forwards_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "right_foot_forwards_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "left_foot_norm_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "right_foot_norm_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "current_joint_angles": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "current_body_position": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_joint_velocities": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "current_body_orientation_quaternion": gym.spaces.Box(-1, 1, shape=(4,), dtype=float),
    "current_angular_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_lin_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "target_forwards_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_joint_torques": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "body_acceleration": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "p": gym.spaces.Box(-1, 1, shape=(2,), dtype=float),
    "r": gym.spaces.Box(0.5, 0.5, shape=(2,), dtype=float)
})
 
obs_terms = lambda env, cycle_time, left_cycle_offset, right_cycle_offset, acceleration: {
    "left_foot_contact": np.sum(WalkingSimulator.foot_contact(1)),
    "right_foot_contact": np.sum(WalkingSimulator.foot_contact(2)), 
    "left_foot_forwards_velocity": np.array([WalkingSimulator.get_left_foot_velocity()[0]], dtype='float64'),
    "right_foot_forwards_velocity": np.array([WalkingSimulator.get_right_foot_velocity()[0]], dtype='float64'),
    "left_foot_norm_velocity": np.array([norm(WalkingSimulator.get_left_foot_velocity())], dtype='float64'),
    "right_foot_norm_velocity": np.array([norm(WalkingSimulator.get_right_foot_velocity())], dtype='float64'),
    "current_joint_angles": np.array(WalkingSimulator.get_joint_angles(), dtype='float64'),
    "current_body_position": np.array(WalkingSimulator.get_6d_pose()[:3], dtype='float64'),
    "current_joint_velocities": np.array(WalkingSimulator.get_joint_velocities(), dtype='float64'),
    "current_body_orientation_quaternion": np.array(WalkingSimulator.get_body_orientation_quaternion(), dtype='float64'),
    "current_angular_velocity": np.array(WalkingSimulator.get_angular_velocity(), dtype='float64'),
    "current_lin_vel": np.array(WalkingSimulator.get_velocity(), dtype='float64'),
    "target_forwards_vel": np.array([0.20, 0, 0]),
    "current_joint_torques": np.array(WalkingSimulator.get_joint_torques(), dtype='float64'),
    "body_acceleration": np.array(acceleration, dtype='float64'),
    "p": np.array([np.sin(2*np.pi*((cycle_time+left_cycle_offset)%1)), np.sin(2*np.pi*((cycle_time+right_cycle_offset)%1))], dtype='float64'),
    "r": np.array([0.5, 0.5], dtype='float64')
    }

rew_terms = [
    lambda _, __, ___, ____: 1, #bias
    lambda _, __, ___, periodic_reward_values: 0.4*np.sum(WalkingSimulator.foot_contact(1) * periodic_reward_values["expected_c_frc_left"]), #left foot force
    lambda _, __, ___, periodic_reward_values: 0.4*np.sum(periodic_reward_values["expected_c_spd_left"] * (1-np.exp(-2*norm(WalkingSimulator.get_left_foot_velocity())**2))), #left foot speed
    lambda _, __, ___, periodic_reward_values: 0.4*np.sum(WalkingSimulator.foot_contact(2) * periodic_reward_values["expected_c_frc_right"]), #right foot force
    lambda _, __, ___, periodic_reward_values: 0.4*np.sum(periodic_reward_values["expected_c_spd_right"] * (1-np.exp(-2*norm(WalkingSimulator.get_right_foot_velocity())**2))), #right foot speed
    lambda _, obs, __, ___: - 0.3 * np.sum((1-np.exp(-2*np.abs(obs['target_forwards_vel'][0]-obs['current_lin_vel'][0])))), #x velocity
    lambda _, obs, __, ___: -0.3 * (1-np.exp(-2*np.abs(obs["current_lin_vel"][1]))), #y velocity
    lambda env, obs, _, __: -0.3 * (1-np.exp(-3*np.sum((1-env.compute_quaternion_difference(obs["current_body_orientation_quaternion"])**2)))), #quaternion difference
    lambda _, __, last_action, ___: -0.1 * np.sum(1-np.exp(-5*norm(last_action))), #action delta
    lambda _, obs, __, ___: -0.1 * np.sum(1-np.exp(-0.05*norm(obs["current_joint_torques"]))), #torques
    lambda _, obs, __, ___: -0.1 * np.sum(1-np.exp(-0.10*(norm(obs["current_angular_velocity"])+obs["body_acceleration"]))), #acceleration
]

action_space = gym.spaces.Box(low=np.array(lower_joint_limits), high=np.array(upper_joint_limits), shape=(10,), dtype=float)
#action_space = gym.spaces.Box(-10, 10, shape=(10,), dtype=float)

action_function = lambda d: (d.tolist())

random_push = {
    "probability": 0.00,
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
    
    multi_input_lstm_policy_config = dict(lstm_hidden_size=128, n_lstm_layers=2)

    recurrent_ppo_config = {
        "policy": "MultiInputLstmPolicy",
        "gae_lambda": 0.95,
        "gamma": 0.99,
        "n_steps": 2048,
        "batch_size": 32,
        "n_epochs": 4,
        "target_kl": 0.042,
        "ent_coef": 0.0015,
        "learning_rate": 0.0001,
        "clip_range": 0.2,
        "use_sde": True,
        "policy_kwargs": multi_input_lstm_policy_config,
        "sde_sample_freq": 4,
        "verbose": 1,
    }

    config = {
        "total_timesteps": 90e6
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
        name="prc_final",
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
        video_length=300,
    )

    obs_key_to_normalize = ["left_foot_forwards_velocity", "right_foot_forwards_velocity", "left_foot_norm_velocity", "right_foot_norm_velocity", "current_joint_angles", "current_body_position", "current_joint_velocities",
            "current_body_orientation_quaternion", "current_angular_velocity", "current_lin_vel",
            "target_forwards_vel", "current_joint_torques", "body_acceleration", "p", "r"]

    env = VecNormalize(
            env,
            clip_obs = 20.0,
            norm_obs_keys = obs_key_to_normalize
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
    model.save("prc_model")
    run.finish()
