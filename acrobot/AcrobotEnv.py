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
        
        self.dt = 0.2 # Delta time (0.2 because that's what Gymnasium's Acrobot uses)
        self.sim_stonefishRL = 0.001 # StonefishRL delta time

        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self.step_counter = 0
        self.MAX_STEPS = 500 # Step limit to activate 'truncated'
        self.GOAL_POSITION = -5.4 # Height the 2nd acrobot joint must reach


    # Stonefish returns linear and angular velocity as a 3 position vector [x, y, z].
    # But some sensors, like encoders, only give a scalar value: the angular velocity
    # around their rotation axis (defined in the XML file).


    # Since this scalar value is not tied to any specific index in the vector, I assign by
    # default to the z position (index 2) of the vector that Stonefish returns to Python: [None, None, value].

    # Therefore, this does NOT mean that the actual rotation is around the Z axis.
    # The rotation axis is the one defined in the XML with the <axis xyz="..."/> tag.
    # In this case, e.g. the Y axis is defined in the tag, but we'll still find the velocity
    # value at the Z position of the vector because that's where StonefishRL puts it by default.

    # That's why, when we take the vector returned by the sensor, we always read the index 2 of the vector,
    # knowing that the value we care about is placed there, even though its real axis depends on how we defined
    # the axis in the XML file.

    def safe_axis(self, value, axis=2):  # axis 0 = x, 1 = y, 2 = z
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
        
        # Build the command dictionary
        if (manual_commands is None): 
            command = {
                "Acrobot/Servo2": {
                    "TORQUE": tau
                }, 
                "Acrobot/Servo": {
                    "TORQUE": 0.0
                }
            }
        else: # In case it's a reset, because we'll also slow down the  Joint1
            command = manual_commands

        # Convert to the format accepted by StonefishRL
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

        torque_value = 7.0 # Torque used for trying to stabilize the acrobot arms
        
        
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
        
        # Keep "action=1" because it doesn't really matter, we're passing manual_commands and it already knows what to do
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

        # Send the position reset
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