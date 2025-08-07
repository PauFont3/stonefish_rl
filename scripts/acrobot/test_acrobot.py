import os, sys
current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from acrobot.AcrobotEnv import AcrobotEnv
from core.launch_stonefish import launch_stonefish_simulator


launch_stonefish_simulator("Resources/acrobot/acrobot_scene.xml")


env = AcrobotEnv()

# Reset de l'entorn
# Si no li passem cap valor com a 'options' agafarà el valor epr defecte '0.2' 
obs, info = env.reset(seed=123)#, options={"low": -0.2, "high": 0.2})
terminated = False
truncated = False

total_reward = 0

while not (terminated or truncated): # Quan un dels 2 sigui True sortira del bucle

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    
    total_reward += reward
    
    print("\n Step: ", env.step_counter)
    print(f"Action: {action} (Torque: {[-7.0, 0.0, 7.0][action]})")
    print("Observation:", obs)
    print("Reward:", reward)
    print("Total Reward:", total_reward)
    print("Terminated:", terminated, "| Truncated:", truncated)

    if terminated:
        print("OBJECTIU ASSOLIT!!! Resetejant entorn...\n")
        observation, info = env.reset()
        total_reward = 0 
    
    if truncated:
        print("TRUNCATED: S'han fet 500 passos")
        obs, info = env.reset()

    terminated = False
    truncated = False

total_reward = 0
            
env.close()
