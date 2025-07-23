from EnvStonefishRL import EnvStonefishRL
import numpy as np
import struct
#from gymnasium import spaces
#import math
import json


class G500Env(EnvStonefishRL):
    def __init__(self, ip="tcp://localhost:5555", max_steps=500):
        super().__init__(ip)

        self.max_steps = max_steps
        self.step_counter = 0
        
        self.sim_stonefishRL = 0.001 # Delta Time de Stonefish.
        self.dt = 0.2 # Delta Time de entorn que aplicarà el Reinforcement Learning.

        self.thrusters = [
            "girona500/ThrusterSurgePort",
            "girona500/ThrusterSurgeStarboard",
            "girona500/ThrusterHeaveBow",
            "girona500/ThrusterHeaveStern",
            "girona500/ThrusterYaw"
        ]
        self.arm_joints = [
            "girona500/Joint1",
            "girona500/Joint2",
            "girona500/Joint3",
            "girona500/Joint4",
            "girona500/Joint5",
        ]


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

        # Fer el reset de la posicio del robot
        robot_name = "Acrobot"
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
                    "TORQUE": 50.0
                }
                #, 
                #"girona500/ThrusterSurgeStarboard": {
                #    "TORQUE": 50.0
                #}
        }
        cmd_string = self.build_command(command)
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)

        obs = self.get_observation()

        terminated = False
        if terminated:
            reward = 0.0 

        truncated = bool(self.step_counter >= self.max_steps)

        info = {} # Mirar si ens caldria o no

        reward = self.calculate_reward()
        
        return obs, reward, terminated, truncated, info


    def get_observation(self):
        return


    def calculate_reward(self):
        return



    def enviar_command_proves(self):
        command = {
                "girona500/ThrusterSurgePort": {
                    "TORQUE": -50.0
                }, 
                "girona500/ThrusterSurgeStarboard": {
                    "TORQUE": -50.0
                }
        }

        cmd_string = self.build_command(command)
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)
