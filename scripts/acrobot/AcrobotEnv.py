import numpy as np
import math
import json

from core.EnvStonefishRL import EnvStonefishRL
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


    # Stonefish retorna la velocitat lineal i angular en un vector de 3 posicions [x, y, z].
    # Però alguns sensors com els encoders només ens donen un valor escalar, la velocitat 
    # angular del seu eix de rotació (definit al fitxer XML).

    # Com que aquest valor escalar no està associat a cap posicio del vector, he assignat
    # per defecte a la posició z (index 2) del vector que retorna Stonefish a Python: [None, None, valor].

    # Per tant, això no vol dir que la rotació realment sigui al voltant de l'eix Z.
    # L'eix de rotació és el que està definit al XML en la comanda <axis xyz="..."/>.
    # En aquest cas, p.ex, l’eix Y és el que està definit a la comanda, però el valor de velocitat 
    # el trobarem igualment a la posició Z del vector perquè és on StonefishRL l’ha posat per defecte.

    # Per això, quan agafem el vector retornat pel sensor, sempre agafem el valor de la posició 2 del vector
    # sabent que allà hi ha el valor que ens interessa, encara que el seu eix real dependrà de com hagim definit
    # l’eix al fitxer XML.
    def safe_axis(self, value, axis=2):  # eix 0 = x, 1 = y, 2 = z
        if isinstance(value, list) and len(value) >= 3:
            v = value[axis]
            return v if v is not None else np.nan
        return value if value is not None else np.nan


    def get_observation(self):
        theta1 = self.state['Acrobot/Encoder']['angle']
        theta2 = self.state['Acrobot/Encoder2']['angle']
        omega1 = self.safe_axis(self.state["Acrobot/Encoder"]["angular_velocity"], axis=2)
        omega2 = self.safe_axis(self.state["Acrobot/Encoder2"]["angular_velocity"], axis=2)
        
        return np.array([
                cos(theta1),
                sin(theta1),
                cos(theta2),
                sin(theta2),
                omega1,
                omega2],
            dtype=np.float32)
    

    def step(self, action, manual_commands=None):

        self.step_counter += 1
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

        terminated = z <= self.GOAL_POSITION

        if terminated:
            reward = 0.0 

        truncated = bool(self.step_counter >= self.MAX_STEPS)
     
        info = {}

        return obs, reward, terminated, truncated, info


    def reduce_velocity_to_reset(self, theta1, theta2):

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