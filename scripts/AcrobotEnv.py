from EnvStonefishRL import EnvStonefishRL
import numpy as np
from gymnasium import spaces
from numpy import pi

class AcrobotEnv(EnvStonefishRL):

    MAX_VEL_1 = 4 * pi
    MAX_VEL_2 = 9 * pi


    def __init__(self):

        super().__init__()

        # Observacions: [angle1, angle2, velocitat1, velocitat2]
        # angle1 = angle del 1r braç --> Limit: -pi a pi
        # angle2 = angle del 2n braç --> Limit: -pi a pi
        # velocitat1 = velocitat del 1r braç --> Limit: -inf a inf
        # velocitat2 = velocitat del 2n braç --> Limit: -inf a inf
        # Li diem que les dades son de tipus float32
        high = np.array(
            #np.array([cos(theta1), sin(theta1), cos(theta2), sin(theta2), omega1, omega2])
            [1.0, 1.0, 1.0, 1.0, self.MAX_VEL_1, self.MAX_VEL_2], dtype=np.float32
        )
        low = -high

        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Discrete(3)
        self.state = None

    #def get_observation(self):
    #    theta1 = self.state['Acrobot/Servo/Encoder/angle'] # 1r Braç
    #    theta2 = self.state['Acrobot/Servo2/Encoder/angle'] # 2n Braç
    #    omega1 = self.state['Acrobot/Servo/Encoder/angular_velocity'] # Velocitat del 1r Braç 
    #    omega2 = self.state['Acrobot/Servo2/Encoder/angular_velocity'] # Velocitat del 2n Braç
    #
    #    return np.array([np.cos(theta1), np.sin(theta1), 
    #                     np.cos(theta2), np.sin(theta2), 
    #                     omega1, omega2], dtype=np.float32)


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
            omega2
        ], dtype=np.float32)

    # Canviarla perque cridi a Step de EnvStonefishRL, 
    # NO pas que ella mateixa ho envii.
    def step(self, action):
        torque_values = [-1.0, 0.0, 1.0]  # Valors de torque segons l'accio acció --> Discrete(3)
        torque = torque_values[action]
        
        #commands = f"CMD:Acrobot/Servo2:TORQUE:{torque};OBS:"
        #obs_dict = self.send_command(commands)
        #obs = self.get_observation()
        
        #self.send_command({
        #    'Acrobot/Servo2': {'TORQUE': torque}
        #})
        self.send_command(f"CMD:Acrobot/Servo2:TORQUE:{torque};OBS:")

        obs = self.get_observation()

        # Per ara ens inventem els valors que retorna l'entorn
        reward = 0.0
        terminated = False
        truncated = False
        info = {}

        # Gymnasium tindrà: observation, reward, terminated, truncated, info = env.step(action)
        # Per tant, aixo ja és el que hem de retornar.
        return obs, reward, terminated, truncated, info

    #def reset(self, seed=None, options=None): # Crida al reset de EnvStonefishRL
        super().reset()

        obs = self.get_observation()
        info = {}

        return obs, info


    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        obs = self.get_observation()
        return obs, {}

