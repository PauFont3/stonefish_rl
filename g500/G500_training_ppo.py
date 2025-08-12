import os, sys
current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from g500.G500Env import G500Env
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

from core.launch_stonefish import launch_stonefish_simulator

launch_stonefish_simulator("Resources/g500/scenarios/girona500_basic.scn")

# Create the training environment
env = G500Env()


# Create the 'logs' folder if it doesn't already exist
log_dir = "./logs/"
os.makedirs(log_dir, exist_ok=True)

# Callback to evaluate and save the best model
eval_callback = EvalCallback(
    env,
    best_model_save_path=log_dir,
    log_path=log_dir,
    eval_freq=5000,          
    deterministic=True,
    render=False
)

# Create the PPO model
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=1_350_000, callback=eval_callback)

# Save the model
model.save("ppo_g500_final")
