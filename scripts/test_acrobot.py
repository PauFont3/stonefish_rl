from AcrobotEnv import AcrobotEnv
import time

env = AcrobotEnv()

# Reset de l'entorn
obs, info = env.reset()
print("Observación inicial:", obs)

# Executa 5 passos de proba
for i in range(5):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"\n--- STEP {i+1} ---")
    print("Action:", action)
    print("Observation:", obs)
    print("Reward:", reward)
    print("Terminated:", terminated, "| Truncated:", truncated)

env.close()
