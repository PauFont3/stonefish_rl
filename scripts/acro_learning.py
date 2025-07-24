import gymnasium as gym

from stable_baselines3 import PPO
#from stable_baselines3.common.env_util import make_vec_env

from AcrobotEnv import AcrobotEnv

env = AcrobotEnv()

# Crea el model PPO
model = PPO("MlpPolicy", env, verbose=1)#, tensorboard_log="./ppo_acrobot_tensorboard/")

# Entrenar el model
model.learn(total_timesteps=50000)

# Guardar el model
model.save("ppo_acrobot_stonefish_50000_5_4__7_0")

# Executar l'entorn amb el model
obs, _ = env.reset()

terminated = False
truncated = False

while not (terminated or truncated):
    action, _states = model.predict(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}")