import sys
sys.path.append('Simulator')

from motoflex_gym import WalkingSimulator
from motoflex_gym.gym_world import MoToFlexEnv

from typing import Any
from typing import Dict

import optuna
from optuna.pruners import MedianPruner
from optuna.samplers import TPESampler
import logging
import gymnasium as gym
import numpy as np
from numpy.linalg import norm
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecVideoRecorder, VecNormalize
import wandb
from wandb.integration.sb3 import WandbCallback
import torch
import torch.nn as nn

N_TRIALS = 40
N_STARTUP_TRIALS = 5
N_EVALUATIONS = 2
N_TIMESTEPS = 150000
EVAL_FREQ = int(N_TIMESTEPS / N_EVALUATIONS)
N_EVAL_EPISODES = 3

AVERAGE_REWARDS_OVER_N_EPISODES = 50

multi_input_policy_config = dict(net_arch=[128, 128, 128])

ppo_config = {
        "policy": "MultiInputPolicy",
        "gae_lambda": 0.95,
        "gamma": 0.99,
        "n_steps": 512,
        "batch_size": 16,
        "n_epochs": 4,
        "ent_coef": 0.025,
        "learning_rate": 0.0001,
        "clip_range": 0.2,
        "use_sde": True,
        "policy_kwargs": multi_input_policy_config,
        "sde_sample_freq": 4,
        "verbose": 1,
}

# For some more explanations, see envtest.ipynb.
obs_space = gym.spaces.Dict({
    "left_foot_contact": gym.spaces.Discrete(2),
    "right_foot_contact": gym.spaces.Discrete(2),
    "left_foot_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "right_foot_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "current_joint_angles": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "current_body_position": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_joint_velocities": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "current_body_orientation_quaternion": gym.spaces.Box(-1, 1, shape=(4,), dtype=float),
    "current_angular_velocity": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_lin_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "target_forwards_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_joint_torques": gym.spaces.Box(-np.inf, np.inf, shape=(10,), dtype=float),
    "body_acceleration": gym.spaces.Box(-np.inf, np.inf, shape=(1,), dtype=float),
    "p": gym.spaces.Box(-1, 1, shape=(2,), dtype=float)
})
 
obs_terms = lambda env, cycle_time, left_cycle_offset, right_cycle_offset, acceleration: {
    "left_foot_contact": np.sum(WalkingSimulator.foot_contact(1)),
    "right_foot_contact": np.sum(WalkingSimulator.foot_contact(2)), 
    "left_foot_velocity": np.array([WalkingSimulator.get_left_foot_velocity()[0]], dtype='float64'),
    "right_foot_velocity": np.array([WalkingSimulator.get_right_foot_velocity()[0]], dtype='float64'),
    "current_joint_angles": np.array(WalkingSimulator.get_joint_angles(), dtype='float64'),
    "current_body_position": np.array(WalkingSimulator.get_6d_pose()[:3], dtype='float64'),
    "current_joint_velocities": np.array(WalkingSimulator.get_joint_velocities(), dtype='float64'),
    "current_body_orientation_quaternion": np.array(WalkingSimulator.get_body_orientation_quaternion(), dtype='float64'),
    "current_angular_velocity": np.array(WalkingSimulator.get_angular_velocity(), dtype='float64'),
    "current_lin_vel": np.array(WalkingSimulator.get_velocity(), dtype='float64'),
    "target_forwards_vel": np.array([0.20, 0, 0]),
    "current_joint_torques": np.array(WalkingSimulator.get_joint_torques(), dtype='float64'),
    "body_acceleration": np.array(acceleration, dtype='float64'),
    "p": np.array([np.sin(2*np.pi*((cycle_time+left_cycle_offset)%1)), np.sin(2*np.pi*((cycle_time+right_cycle_offset)%1))], dtype='float64')
    }

rew_terms = [
    lambda _, obs, __, ___: -10 * np.sum(0.30-obs["current_body_position"][0]),
]
action_space = gym.spaces.Box(low=-1, high=1, shape=(10,), dtype=float)

action_function = lambda d: (d.tolist())

random_push = {
    "probability": 0.00,
    "force_range_x": [500, 1000]
}

def sample_ppo_params(trial: optuna.Trial) -> Dict[str, Any]:
    gamma = 1.0 - trial.suggest_float("gamma", 0.0001, 0.1, log=True)
    max_grad_norm = trial.suggest_float("max_grad_norm",0.3, 5.0, log=True)
    gae_lambda = trial.suggest_float("gae_lambda", 0.001, 0.2, log=True)
    n_steps = 2**trial.suggest_int("exponent_n_steps", 5, 10)
    learning_rate = trial.suggest_float("lr", 0.0001, 0.003)
    ent_coef = trial.suggest_float("ent_coeff", 0.001, 0.1, log=True)
    ortho_init = trial.suggest_categorical("ortho_init", [False,True])
    net_arch = trial.suggest_categorical("net_arch", ["small", "medium", "large"])
    activation_fn = trial.suggest_categorical("activation_fn", ["tanh", "relu"])

    if(net_arch == "small"):
        net_arch = [128, 128, 128,]
    if(net_arch == "medium"):
        net_arch = [128, 128, 128, 128]
    if(net_arch == "large"):
        net_arch = [128, 300, 300, 128]

    activation_fn = {"tanh": nn.Tanh, "relu": nn.ReLU}[activation_fn]

    return {
            "gamma": gamma,
            "max_grad_norm": max_grad_norm,
            "gae_lambda": gae_lambda,
            "n_steps": n_steps,
            "learning_rate": learning_rate,
            "ent_coef": ent_coef,
            "policy_kwargs": {
                "ortho_init": ortho_init,
                "net_arch": net_arch,
                "activation_fn": activation_fn
            },
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

def objective(trial: optuna.Trial) -> float:
    kwargs = ppo_config
    kwargs.update(sample_ppo_params(trial))
    
    config = {
        "total_timesteps": N_TIMESTEPS
    }

    all_configs = {
        "learn_conf": config,
        "ppo_config": kwargs,
        "reward_terms": rew_terms,
        "observation_space": obs_space,
        "observation_terms": obs_terms,
        "action_space": action_space,
        "action_function": action_function
    }
    
    run = wandb.init(
        name="ppo_potential_based",
        project="sb3",
        config=all_configs,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
        monitor_gym=True,  # auto-upload the videos of agents playing the game
        save_code=True,  # optional
    )

    env = DummyVecEnv([make_env]) 

    env = VecVideoRecorder(
        env,
        f"tmp/videos/{run.id}",
        record_video_trigger=lambda x: x % 30000 == 0,
        video_length=200,
    )
    
    obs_key_to_normalize = ["left_foot_velocity", "right_foot_velocity", "current_joint_angles", "current_body_position", "current_joint_velocities",
            "current_body_orientation_quaternion", "current_angular_velocity", "current_lin_vel",
            "target_forwards_vel", "current_joint_torques", "body_acceleration", "p"]

    env = VecNormalize(
            env,
            clip_obs = 20.0,
            norm_obs_keys = obs_key_to_normalize
    )

    model = PPO(
        env=env,
        **kwargs,
        tensorboard_log=f"tmp/runs/{run.id}"
    )    

    nan_encountered = False
    value_error = False
    try:
        model.learn(
            **config,
            callback=WandbCallback(
                gradient_save_freq=100,
                model_save_path=f"tmp/models/{run.id}",
                verbose=2,
            ),
        )
    except AssertionError as e:
        print(e)
        nan_encountered = True
    except ValueError as e:
        print(e)
        value_error = True
    finally:
        model.env.close()

    if nan_encountered:
        return float("nan")

    if value_error:
        return float("nan")

    run.finish()
    
    rewards = env.env_method("get_episode_rewards")
    last_n_episode_rewards = rewards[0][-50::1]
    average_rewards_for_last_n_episodes = sum(last_n_episode_rewards)/AVERAGE_REWARDS_OVER_N_EPISODES
    
    return average_rewards_for_last_n_episodes
    
if __name__ == "__main__":


    # Add stream handler of stdout to show the messages
    optuna.logging.get_logger("optuna").addHandler(logging.StreamHandler(sys.stdout))
    study_name = "pbr_optuna"  # Unique identifier of the study.
    storage_name = "sqlite:///{}.db".format(study_name)

    sampler = TPESampler(n_startup_trials=N_STARTUP_TRIALS)
    pruner = MedianPruner(n_startup_trials=N_STARTUP_TRIALS, n_warmup_steps = 1e5)

    study = optuna.create_study(sampler=sampler, pruner=pruner, direction="maximize", study_name=study_name, storage=storage_name, load_if_exists=True)
    
    try:
        study.optimize(objective, n_trials=N_TRIALS, timeout=None)
    except KeyboardInterrupt:
        pass
