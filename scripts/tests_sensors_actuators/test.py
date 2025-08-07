import os, sys
current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from tests_sensors_actuators.G500TestEnv import G500TestEnv
from core.launch_stonefish import launch_stonefish_simulator


launch_stonefish_simulator("Resources/tests_sensors_actuators/scenarios/girona500_basic.scn")

env = G500TestEnv()

obs, info = env.reset()

while True:
    action = env.action_space.sample() 
    obs, reward, terminated, truncated, info = env.step(action)   
    print("\n\n")
                        