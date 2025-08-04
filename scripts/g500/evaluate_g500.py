import os, sys

current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from g500.G500Env import G500Env
from stable_baselines3 import PPO
from core.launch_stonefish import launch_stonefish_simulator

launch_stonefish_simulator("Resources/udg_cirs-iauv_simulation/scenarios/girona500_basic.scn")

model = PPO.load("logs/best_model.zip")  

env = G500Env()  
obs, _ = env.reset()


done = False
truncated = False

while not done and not truncated:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)

    print(f"Reward: {reward}")

if truncated:
    print("Truncated")
else:
    print("Terminated")