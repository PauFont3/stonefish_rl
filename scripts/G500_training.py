from stable_baselines3 import PPO
from G500Env import G500Env

env = G500Env()

# Crear model
model = PPO("MlpPolicy", env, verbose=1)

# Entrenar model
model.learn(total_timesteps=50000)

obs, _ = env.reset()

# Guardar model
model.save("ppo_g500")