import os, sys
current_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

from g500.G500Env import G500Env
from core.launch_stonefish import launch_stonefish_simulator


launch_stonefish_simulator("Resources/tests_sensors_actuators/scenarios/girona500_basic.scn")

env = G500Env()

obs, info = env.reset()

while True:

    action = env.action_space.sample() # Vector amb valors aleatoris de -1 a 1 de 11 floats 
    obs, reward, terminated, truncated, info = env.step(action)   
                        