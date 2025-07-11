from EnvStonefishRL import EnvStonefishRL
import numpy as np
from gymnasium import spaces
from numpy import pi

class AcrobotEnv(EnvStonefishRL):

    MAX_VEL_1 = 4 * pi
    MAX_VEL_2 = 9 * pi


    def __init__(self):

        super().__init__()

        high = np.array(
        #   [cos(theta1), sin(theta1), cos(theta2), sin(theta2), omega1, omega2]
            [1.0, 1.0, 1.0, 1.0, self.MAX_VEL_1, self.MAX_VEL_2], dtype=np.float32
        )
        low = -high
        
        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self.state = None

        # Constants per saber si l'agent guanyat
        self.LINK_LENGTH_1 = 1.0
        self.LINK_LENGTH_2 = 1.0


    def get_observation(self):
        
        theta1 = self.state['Acrobot/Encoder']['angle']
        theta2 = self.state['Acrobot/Encoder2']['angle']
        omega1 = self.state['Acrobot/Encoder']['angular_velocity']
        omega2 = self.state['Acrobot/Encoder2']['angular_velocity']
    
        return np.array([
                np.cos(theta1),
                np.sin(theta1),
                np.cos(theta2),
                np.sin(theta2),
                omega1,
                omega2],
            dtype=np.float32)
    
       
    # Canviarla perque cridi a Step de EnvStonefishRL?
    # O que ella mateixa ho envii.
    def step(self, action):

        torque = [-1000.0, -1000.0, -1000.0]
        tau = torque[action]
        
        # Construir el diccionari de comandes
        command = {
            "Acrobot/Servo2": {
                "TORQUE": tau
            }, 
            "Acrobot/Servo": {
                "TORQUE": 0.1
            }
        }
        # Convertir al format que accepta StonefishRL
        cmd_string = self.build_command(command)
        super().send_command(cmd_string)
        
        obs = self.get_observation()

        reward = -1.0 

        theta1 = np.arctan2(obs[1], obs[0])
        theta2 = np.arctan2(obs[3], obs[2])

        # Funció que dona el reward
        height = -self.LINK_LENGTH_1 * np.cos(theta1) - self.LINK_LENGTH_2 * np.cos(theta1 + theta2)
        #terminated = bool(height > self.LINK_LENGTH_1)
        terminated = False
        truncated = False
        info = {}

        return obs, reward, terminated, truncated, info


    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        obs = self.get_observation()
        info = {}
        return obs,info
