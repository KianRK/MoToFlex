from gymnasium.envs.registration import register

register(
    id="MoToFlex/WalkingSimulator-v0",
    entry_point="motoflex_gym.gym_world:MoToFlexEnv",
)