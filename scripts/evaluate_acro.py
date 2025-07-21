from stable_baselines3 import PPO
from AcrobotEnv import AcrobotEnv

env = AcrobotEnv()

# Carregar el model entrenat
model = PPO.load("ppo_acrobot_stonefish_50000_5_4__7_0",env=env)

# Evaluar durant N episodis
n_episodes = 5

for episode in range(n_episodes):
    obs, _ = env.reset()
    terminated = False
    truncated = False
    step = 0
    print(f"\n--- Episodi {episode + 1} ---")

    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)        

        step += 1
        print(f"Step {step} | Reward: {reward:.2f} | Terminated: {terminated} | Truncated: {truncated}")
