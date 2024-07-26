import sys
sys.path.append('Simulator')

import motoflex_gym

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecVideoRecorder
import wandb
from wandb.integration.sb3 import WandbCallback


config = {
    "policy_type": "MlpPolicy",
    "total_timesteps": 1e5,
    "env_name": "MoToFlex-PPO",
}

run = wandb.init(
    name="MoToFlex-PPO",
    project="sb3",
    config=config,
    sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
    monitor_gym=True,  # auto-upload the videos of agents playing the game
    save_code=True,  # optional
)


def make_env():
    env = gym.make(config["env_name"], render_mode='rgb_array')
    env = Monitor(env)  # record stats such as returns
    return env


env = DummyVecEnv([make_env])
env = VecVideoRecorder(
    env,
    f"tmp/videos/{run.id}",
    record_video_trigger=lambda x: x % 2000 == 0,
    video_length=200,
)
model = PPO(
    config["policy_type"],
    env,
    gae_lambda=0.95,
    gamma=0.9,
    n_epochs=10,
    ent_coef=0.0,
    learning_rate=1e-3,
    clip_range=0.2,
    use_sde=True,
    sde_sample_freq=4,
    verbose=1,
    tensorboard_log=f"tmp/runs/{run.id}"
    )

model.learn(
    total_timesteps=config["total_timesteps"],
    callback=WandbCallback(
        gradient_save_freq=100,
        model_save_path=f"tmp/models/{run.id}",
        verbose=2,
    ),
)
run.finish()