import os, sys
current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from g500.G500Env import G500Env
from core.launch_stonefish import launch_stonefish_simulator


launch_stonefish_simulator("Resources/g500/scenarios/girona500_basic.scn")
 

env = G500Env()

obs, info = env.reset()
terminated = False
truncated = False
total_reward = 0

while not (terminated or truncated):

    action = env.action_space.sample() # Vector amb valors aleatoris pels 11 floats dels actuadors
    obs, reward, terminated, truncated, info = env.step(action)   
    
    print("\n Step: ", env.step_counter)
    print("Observation:", obs)
    print("Reward:", reward)
    print("Terminated:", terminated, "| Truncated:", truncated)

    if terminated:
        print("OBJECTIU ASSOLIT :) Resetejant entorn...\n")
        observation, info = env.reset()
        
    if truncated:
        print("TRUNCATED: S'han superat els 30 segs :(")
        observation, info = env.reset()

    terminated = False
    truncated = False
