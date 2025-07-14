import time
from AcrobotEnv import AcrobotEnv

env = AcrobotEnv()

# Reset de l'entorn
obs, info = env.reset()
print("Observación inicial:", obs)

total_reward = 0
for i in range(5000):

    # Escull una opció aleatoria
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    
    total_reward += reward

    print(f"\n--- STEP {i+1} ---")
    print(f"Action: {action} (Torque: {[-1000.0, 0.0, 1000.0][action]})")
    print("Observation:", obs)
    print("Reward:", reward)
    print("Total Reward:", total_reward)
    print("Terminated:", terminated, "| Truncated:", truncated)
   
    
    if terminated: #or truncated:
        print("\n OBJECTIU ASSOLIT! Resetejant entorn...")
        observation, info = env.reset()
        total_reward = 0 
        #time.sleep(0.1)
        
env.close()
