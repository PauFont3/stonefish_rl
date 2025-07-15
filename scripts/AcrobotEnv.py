from EnvStonefishRL import EnvStonefishRL
import numpy as np
from gymnasium import spaces
from numpy import cos, pi, sin, arctan2

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
        
        self.dt = 0.2 # Delta Time (0.2 pq es el que hi ha al acrobot fet pels de gymnasium)
        self.sim_stonefishRL = 0.001 # El Delta Time del StonefishRL

        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        #self.state = None
 
        # Constants per saber si l'agent guanyat
        self.LINK_LENGTH_1 = 1.0
        self.LINK_LENGTH_2 = 1.0

        self.step_counter = 0
        self.MAX_STEPS = 500 # Limit de steps per activar 'truncated'


    def get_observation(self):
        
        theta1 = self.state['Acrobot/Encoder']['angle']
        print("Valor theta1: ", self.state['Acrobot/Encoder']['angle'])
        theta2 = self.state['Acrobot/Encoder2']['angle']
        print("Valor theta2: ", self.state['Acrobot/Encoder2']['angle'])
        omega1 = self.state['Acrobot/Encoder']['angular_velocity']
        print("Valor omega2: ", omega1)
        omega2 = self.state['Acrobot/Encoder2']['angular_velocity']
        print("Valor omega2: ", omega2)

        return np.array([
                    
                cos(theta1),
                sin(theta1),
                cos(theta2),
                sin(theta2),
                omega1,
                omega2],
            dtype=np.float32)
    
       
    # Canviarla perque cridi a Step de EnvStonefishRL?
    # O que ella mateixa ho envii.
    def step(self, action):

        self.step_counter += 1
        torque = [-1000.0, 0.0, 1000.0]

        tau = torque[action]
        
        # Construir el diccionari de comandes
        command = {
            "Acrobot/Servo2": {
                "TORQUE": tau
            }, 
            "Acrobot/Servo": {
                "TORQUE": 0.0
            }
        }

        if self.sim_stonefishRL <= 0:
            raise ValueError("El valor de sim_stonefishRL ha de ser >0")

        # Convertir al format que accepta StonefishRL
        cmd_string = self.build_command(command) 
        
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)
        
        obs = self.get_observation()

        reward = -1.0 

        theta1 = arctan2(obs[1], obs[0])
        theta2 = arctan2(obs[3], obs[2])

        # Funció que dona el reward
        height = -self.LINK_LENGTH_1 * cos(theta1) - self.LINK_LENGTH_2 * cos(theta1 + theta2)
        #terminated = bool(-cos(theta1) - cos(theta2 + theta1) > 1.9)
        #terminated = bool(-self.LINK_LENGTH_1 * cos(theta1) - self.LINK_LENGTH_2 * cos(theta1 + theta2) > 0.5)

        terminated = bool(height > self.LINK_LENGTH_1)

        #valor = -self.LINK_LENGTH_1 * cos(theta1) - self.LINK_LENGTH_2 * cos(theta1 + theta2)
        valor_terminated = (-cos(theta1) - cos(theta2 + theta1))
        print("Valor del Step Counter: ", self.step_counter)
        print("Valor step per acceptar el 'TERMINATED': ", valor_terminated)
        #terminated = bool(height > self.LINK_LENGTH_1)
        
        #terminated = False
        if terminated:
            reward = 0.0 # Recompensa, pq representa que ha arribat al objectiu
        
        # S'activa quan portem >= 500 steps a Python.
        # Utilitzo la version history (v1). D'aqui ve la limitació de les 500 steps
        truncated = bool(self.step_counter >= self.MAX_STEPS)
        #truncated = False
        info = {}

        return obs, reward, terminated, truncated, info


    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_counter = 0
        obs = self.get_observation()
        info = {}
        return obs,info