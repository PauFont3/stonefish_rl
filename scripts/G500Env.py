from EnvStonefishRL import EnvStonefishRL
import numpy as np
import struct
import math
#from gymnasium import spaces
#import math
import json


class G500Env(EnvStonefishRL):
    def __init__(self, ip="tcp://localhost:5555", search_time=20):
        super().__init__(ip)

        self.step_counter = 0
        
        self.ok_dist_gripper_object = 0.5
        self.search_time = search_time # Temps que volem que el robot estigui explorant

        self.sim_stonefishRL = 0.001 # Delta Time de Stonefish.
        self.dt = 0.2 # Delta Time de l'entorn que aplica el Reinforcement Learning.


    def reset(self, seed=None, options=None):

        if options is None:
            options = {}

        low = -0.2
        high = 0.2 

        x = self.np_random.uniform(low, high)
        y = self.np_random.uniform(low, high)
        z = self.np_random.uniform(low, high)
        rot_x = self.np_random.uniform(low, high)
        rot_y = self.np_random.uniform(low, high)
        rot_z = self.np_random.uniform(low, high)

        valors = [x, y, z, rot_x, rot_y, rot_z]       

        tipus_format = "f" * len(valors) # Posa a float tots els valors que hi ha al vector 'valors'
        novesPosicions = struct.pack(tipus_format, *valors)

        robot_name = "Ball" # Nom del robot al qual se li aplicarà el reset amb una nova posició random
        obs = self.send_command("RESET:" + robot_name + ";")
        self.socket.send(novesPosicions)

        super().reset(seed=seed)
        
        self.step_counter = 0
        obs = self.get_observation()
        info = {}

        return obs, info

        
    def step(self, action):

        self.step_counter += 1

        # Canviar els valors segons necessitats
        torque = [-50.0, 0.0, 50.0]
        tau = torque[action]

        command = {
                "girona500/ThrusterSurgePort": {
                "TORQUE": -5.0
            },
            #"girona500/ThrusterHeaveStern": {
            #    "TORQUE": -100.0
            #}
            #,
            # "girona500/ThrusterHeaveBow": {
            #    "TORQUE": -100.0
            #}
            #,
            "girona500/Servo1": {
                "TORQUE": 0.1
            },
            "girona500/Servo2": {
                "TORQUE": 0.1
            },
            "girona500/Servo3": {
                "TORQUE": 0.1
            },
            "girona500/Servo4": {
                "TORQUE": 0.1
            },
            "girona500/FingerServo2": {
                "TORQUE": 0.1
            },
            "girona500/ThrusterSurgeStarboard": {
                "TORQUE": -3.0
            }
        }

        cmd_string = self.build_command(command)
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)

        obs = self.get_observation()
        reward = self.calculate_reward() 

        print("Dist: ", reward *-1) # *-1 pq el reward és la distancia però en negatiu, cal canviar a posotiu per aconseguir la distancia 

        terminated = False 
        truncated = False

        if(reward == 0): # Si el reward és 0, representa que estem a > 0.5 de distància i que ja hem arribat a l'objectiu
            terminated = True

        elif (self.step_counter * self.dt >= self.search_time):
            truncated = True

        info = {}

        return obs, reward, terminated, truncated, info


    def get_observation(self):
        return


    def calculate_reward(self):
        reward = 0
        actual_dist = self.dist_gripper_object()
        if(self.ok_dist_gripper_object < actual_dist):
            reward = actual_dist *-1

        return reward


    #def enviar_command_proves(self):
        command = {
            "girona500/ThrusterSurgePort": {
                "TORQUE": -5.0
            },
            #"girona500/ThrusterHeaveStern": {
            #    "TORQUE": -100.0
            #}
            #,
            # "girona500/ThrusterHeaveBow": {
            #    "TORQUE": -100.0
            #}
            #,
            "girona500/Servo1": {
                "TORQUE": 0.1
            },
            "girona500/Servo2": {
                "TORQUE": 0.1
            },
            "girona500/Servo3": {
                "TORQUE": 0.1
            },
            "girona500/Servo4": {
                "TORQUE": 0.1
            },
            "girona500/FingerServo2": {
                "TORQUE": 0.1
            },
            "girona500/ThrusterSurgeStarboard": {
                "TORQUE": -3.0
            }
        }

        cmd_string = self.build_command(command)
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)
    

    def dist_gripper_object(self):
        
        vec_xyz_ball = self.state['Ball']['position']
        vec_xyz_gripper = self.state['girona500/OdoGripper']['position']
        #rot_xyz_gripper = self.state['girona500/OdoGripper']['rotation']

        dist = np.linalg.norm(np.array(vec_xyz_gripper) - np.array(vec_xyz_ball))
        return dist