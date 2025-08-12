import os, sys
current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from acrobot.AcrobotEnv import AcrobotEnv
from stable_baselines3 import PPO
from core.launch_stonefish import launch_stonefish_simulator

launch_stonefish_simulator("Resources/acrobot/acrobot_scene.xml")


env = AcrobotEnv()

# Create the PPO model
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=50000)

# Save the model
model.save("ppo_acrobot_stonefish_50000_5_4__7_0")

# Run the environment with the model
obs, _ = env.reset()

terminated = False
truncated = False

while not (terminated or truncated):
    action, _states = model.predict(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}")