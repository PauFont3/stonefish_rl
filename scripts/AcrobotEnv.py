from EnvStonefishRL import EnvStonefishRL
import numpy as np
import math
import json
from gymnasium import spaces
from numpy import cos, pi, sin

class AcrobotEnv(EnvStonefishRL):

    MAX_VEL_1 = 4 * pi
    MAX_VEL_2 = 9 * pi


    def __init__(self, ip="tcp://localhost:5555"):

        super().__init__(ip)
        
        high = np.array(
        #   [cos(theta1), sin(theta1), cos(theta2), sin(theta2), omega1, omega2]
            [1.0, 1.0, 1.0, 1.0, self.MAX_VEL_1, self.MAX_VEL_2], dtype=np.float32
        )
        low = -high
        
        self.dt = 0.2 # Delta Time (0.2 pq es el que hi ha al acrobot fet pels de gymnasium)
        self.sim_stonefishRL = 0.001 # El Delta Time del StonefishRL

        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self.step_counter = 0
        self.MAX_STEPS = 500 # Limit de steps per activar 'truncated'
        self.GOAL_POSITION = -5.4 # Altura a la que ha d'arribar la 2a articulació de l'acrobot


    def get_observation(self):
        
        theta1 = self.state['Acrobot/Encoder']['angle']
        #print("Valor theta1: ", self.state['Acrobot/Encoder']['angle'])
        theta2 = self.state['Acrobot/Encoder2']['angle']
        #print("Valor theta2: ", self.state['Acrobot/Encoder2']['angle'])
        omega1 = self.state['Acrobot/Encoder']['angular_velocity']
        #print("Valor omega2: ", omega1)
        omega2 = self.state['Acrobot/Encoder2']['angular_velocity']
        #print("Valor omega2: ", omega2)

        #z = self.state["Acrobot/Odometry"]["position"][2]
        #print("Posició eix 'Z' del sensor Odometry: ", z)

        # Normalitzar angles a [-pi, pi]
        #theta1 = np.fmod(theta1 + pi, 2 * pi) - pi
        #theta2 = np.fmod(theta2 + pi, 2 * pi) - pi

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
    def step(self, action, manual_commands=None):

        self.step_counter += 1
        #print(self.step_counter)
        torque = [-7.0, 0.0, 7.0]

        tau = torque[action]
        
        # Construir el diccionari de comandes
        if (manual_commands is None): 
            command = {
                "Acrobot/Servo2": {
                    "TORQUE": tau
                }, 
                "Acrobot/Servo": {
                    "TORQUE": 0.0
                }
            }
        else: # En cas que sigui un reset, perque frenarem també la Joint1
            command = manual_commands

        # Convertir al format que accepta StonefishRL
        cmd_string = self.build_command(command) 
        
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)
        
        obs = self.get_observation()

        reward = -1.0 

        z = self.state["Acrobot/Odometry"]["position"][2]

        #print("Valor del Step Counter: ", self.step_counter)
        
        terminated = z <= self.GOAL_POSITION

        if terminated:
            reward = 0.0 

        truncated = bool(self.step_counter >= self.MAX_STEPS)
     
        info = {}

        return obs, reward, terminated, truncated, info


    def reduce_velocity_to_reset(self, theta1, theta2):

        #torque_value = 7.0 # Valor de força amb el que intentarem estabilitzar els braços de l'acrobot
        torque_value = 7.0 # Valor de força amb el que intentarem estabilitzar els braços de l'acrobot
        
        llindar_error = 0.3
        if theta1 < -llindar_error:
            tau1 = torque_value
        elif theta1 > llindar_error:
            tau1 = -torque_value
        else:
            tau1 = 0.0

        if theta2 < -llindar_error:
            tau2 = torque_value
        elif theta2 >= llindar_error:
            tau2 = -torque_value
        else:
            tau2 = 0.0            

        command = {
            "Acrobot/Servo": {
                "TORQUE": tau1
            },
            "Acrobot/Servo2": {
                "TORQUE": tau2
            }
        }
        
        # Deixem "action=1" perque realment no ens afecta, ja estem passant el manual_commands i ja sap el que haura de fer
        obs, reward, terminated, truncated, info = self.step(action=1, manual_commands=command)
        return obs


    def reset(self, seed=None, options=None):
        
        if options is None:
            options = {}

        low = options.get("low", -0.2)
        high = options.get("high", 0.2) 

        x = self.np_random.uniform(low, high)
        y = self.np_random.uniform(low, high)
        z = -4.0
        roll = self.np_random.uniform(low, high)
        pitch = self.np_random.uniform(low, high)
        yaw = -3.14

        reset_dict = [{
            "name": "Acrobot",
            "position": [x, y, z],
            "rotation": [roll, pitch, yaw]
        }]

        # Fer el reset de la posicio
        obs = self.send_command("RESET:" + json.dumps(reset_dict) + ";")
        
        super().reset(obs, seed=seed, options=options)
    
        n_steps = 100

        obs = self.get_observation()
        for _ in range(n_steps): 

            cos_theta1 = obs[0]
            sin_theta1 = obs[1]
            cos_theta2 = obs[2]
            sin_theta2 = obs[3]

            theta1 = math.atan2(sin_theta1, cos_theta1)
            theta2 = math.atan2(sin_theta2, cos_theta2)

            self.reduce_velocity_to_reset(theta1, theta2)
            obs = self.get_observation()
            
        self.step_counter = 0
        obs = self.get_observation()
        info = {}
        return obs,info