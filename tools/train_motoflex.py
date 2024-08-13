import sys
sys.path.append('Simulator')

from motoflex_gym import WalkingSimulator
from motoflex_gym.gym_world import MoToFlexEnv

import json
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecVideoRecorder
import wandb
from wandb.integration.sb3 import WandbCallback
from bayes_opt import BayesianOptimization

# For some more explanations, see envtest.ipynb.
obs_space = gym.spaces.Dict({
    "current_body_pos": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_lin_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "current_body_orientation": gym.spaces.Box(-1.57, 1.57, shape=(3,), dtype=float),
    "current_polar_coords": gym.spaces.Box(-np.inf, np.inf, shape=(6,), dtype=float),
    "target_com_pos": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
    "target_lin_vel": gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=float),
})

obs_terms = lambda: {
    "current_body_pos": np.array(WalkingSimulator.get_6d_pose()[:3]),
    "current_lin_vel": np.array(WalkingSimulator.get_velocity()),
    "current_polar_coords": MoToFlexEnv.current_polar_pos(),
    "current_body_orientation": np.array(WalkingSimulator.get_6d_pose()[3:]),
    "target_com_pos": np.array([0, 0, 0.34], dtype='float64'),
    "target_lin_vel": np.array([0.05, 0, 0], dtype='float64')
    }


action_space = gym.spaces.Box(-10, 10, shape=(6,), dtype=float)

action_function = lambda d, env: (env.delta_polar_to_angles(*d / 100), d.tolist())

random_push = {
    "probability": 0.01,
    "force_range_x": [500, 1000]
}

def make_rew_terms(bias, height_weight, vel_weight, eff_weight, dir_weight):
    
    return [ 
        lambda _, __: bias,
        lambda obs, _: - height_weight * np.sum(np.abs(obs["current_body_pos"][2] - obs["target_com_pos"][2])),
        lambda obs, _: - vel_weight * np.sum(np.abs(obs['current_lin_vel'] - obs['target_lin_vel'])),
        lambda _, last_action: - eff_weight * np.sum(np.abs(last_action)),
        lambda obs, _: - dir_weight * np.sum(np.abs(obs['current_body_orientation'][:2]))
    ]



def make_env(w1, w2, w3, w4, w5):    

    rew_terms = make_rew_terms(w1, w2, w3, w4, w5)

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

def train(bias, height_weight, vel_weight, eff_weight, dir_weight):
  
    rew_terms = make_rew_terms(bias, height_weight, vel_weight, eff_weight, dir_weight)

    ppo_config = {
        "policy": "MultiInputPolicy",
        "gae_lambda": 0.95,
        "gamma": 0.9,
        "n_epochs": 10,
        "ent_coef": 0.0,
        "learning_rate": lambda x: x * 1e-4 + (1 - x) * 1e-6,
        "clip_range": 0.2,
        "use_sde": True,
        "sde_sample_freq": 4,
        "verbose": 1,
    }

    config = {
        #"total_timesteps": 6e7
        "total_timesteps": 5e6
    }

    all_configs = {
        "learn_conf": config,
        "ppo_config": ppo_config,
        "reward_terms": rew_terms,
        "observation_space": obs_space,
        "observation_terms": obs_terms,
        "action_space": action_space,
        "action_function": action_function
    }
    
    run = wandb.init(
        name="1_orf_target_vel:50_woptim",
        project="sb3",
        config=all_configs,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
        monitor_gym=True,  # auto-upload the videos of agents playing the game
        save_code=True,  # optional
    )

    env = make_env(bias, height_weight, vel_weight, eff_weight, dir_weight)
    #if True:
     #   env = SubprocVecEnv([make_env for _ in range(20)])
    #else:
     #   env = DummyVecEnv([make_env])

    '''env = VecVideoRecorder(
        env,
        f"tmp/videos/{run.id}",
        record_video_trigger=lambda x: x % 30000 == 0,
        video_length=200,
    )'''
    model = PPO(
        env=env,
        **ppo_config,
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
    
    episode_rewards = env.get_episode_rewards()

    mean_reward_per_episode = sum(episode_rewards) / len(episode_rewards)
    

    return mean_reward_per_episode

if __name__ == "__main__":
    pbounds = {'bias': (2, 5), 'height_weight': (25, 150), 'vel_weight': (1, 15), 'eff_weight': (0.01, 0.25), 'dir_weight': (1, 25)}
    optimizer = BayesianOptimization(
            f=train,
            pbounds=pbounds,
            verbose=2,
            random_state=1,
            )

    optimizer.maximize(
            init_points=5,
            n_iter=10,
            )

    with open('max_params.txt', 'w') as file:
        file.write(json.dumps(optimizer.max, indent=4))
