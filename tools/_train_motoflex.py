import sys
sys.path.append('Simulator')

from motoflex_gym import WalkingSimulator
from motoflex_gym.gym_world import MoToFlexEnv

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecVideoRecorder
import wandb
from wandb.integration.sb3 import WandbCallback

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

def rew_terms(sweep_conf):
    try:
        return [ 
                lambda _, __: sweep_conf.bias,
                lambda obs, _: - sweep_conf.height_weight * np.sum(np.abs(obs["current_body_pos"][2] - obs["target_com_pos"][2])),
                lambda obs, _: - sweep_conf.vel_weight * np.sum(np.abs(obs['current_lin_vel'] - obs['target_lin_vel'])),
                lambda _, last_action: - sweep_conf.eff_weight * np.sum(np.abs(last_action)),
                lambda obs, _: - sweep_conf.dir_weight * np.sum(np.abs(obs['current_body_orientation'][:2]))
            ]
    except:
        print("Problem in rew_terms")

action_space = gym.spaces.Box(-10, 10, shape=(6,), dtype=float)

action_function = lambda d, env: (env.delta_polar_to_angles(*d / 100), d.tolist())

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
        "total_timesteps": 2e7
    }


random_push = {
    "probability": 0.01,
    "force_range_x": [500, 1000]
}

def make_env(sweep_conf):
    env = gym.make("MoToFlex/WalkingSimulator-v0", 
                render_mode='rgb_array', 
                observation_space = obs_space,
                observation_terms = obs_terms,
                reward_functions = rew_terms(sweep_conf),
                action_space = action_space,
                action_function = action_function,
                ab_filter_alpha = 0.1,
                random_push = random_push
                )
    env = Monitor(env)  # record stats such as returns
    return env

def train():

#    rew_functions = rew_terms(sweep_conf)

    try:
        run = wandb.init(
            name="1_orf_tv:50_sweep1",
            sync_tensorboard=True,
	    monitor_gym=True,
	    save_code=True
	    ) 
    except:
        print("Init did not work")

#    try:
 #       if True:
  #          env = SubprocVecEnv([make_env(wandb.config) for _ in range(20)])
   #         print("SUBPROCENV")
    #    else:
     #       env = DummyVecEnv([make_env(config.bias, config.height_weight, config.vel_weight, config.eff_weight, config.dir_weight)])
      #      print("DUMMYPROCENV")
   # except:
    #    print("make_env did not work")
    
    env = make_env(wandb.config)

    env = VecVideoRecorder(
        env,
        f"tmp/videos/{run.id}",
        record_video_trigger=lambda x: x % 30000 == 0,
        video_length=200,
    )

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


if __name__ == "__main__":

    sweep_config = {

        "method": "bayes",
        "metric": {
            "name": "reward",
            "goal": "maximize"
        },
        "parameters": {
            "bias": {
                "min": 1,
                "max": 5
            },
            "height_weight": {
                "min": 30,
                "max": 130
            },
            "vel_weight": {
                "min": 0.5,
                "max": 10.0
            },
            "eff_weight": {
                "min": 0.05,
                "max": 0.2
            },
            "dir_weight": {
                "min": 1,
                "max": 10
            }
        }
    }

    sweep_id = wandb.sweep(sweep=sweep_config, project="sb3")
    wandb.agent(sweep_id, function=train, count=10)

