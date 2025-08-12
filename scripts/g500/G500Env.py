import numpy as np
import json

from core.EnvStonefishRL import EnvStonefishRL
from gymnasium import spaces


class G500Env(EnvStonefishRL):
    def __init__(self, ip="tcp://localhost:5555", search_time=30):
        super().__init__(ip)

        self.step_counter = 0
        
        self.ok_dist_gripper_object = 0.5 # Acceptable distance between the object and the gripper
        self.search_time = search_time # Time the robot is allowed to search

        self.sim_stonefishRL = 0.001 # StonefishRL delta time.
        self.dt = 0.2 # Environment delta time used by Reinforcement Learning.


        # For the observations
        ball_position = robot_position = robot_rotation = 3
        joints = 6  # 4 servos + 2 finger servos
        robot_linear_vel = robot_angular_vel = 3
        pos_gripper = rot_gripper = 3
        n_total_obs = ball_position + robot_position + robot_rotation + joints + robot_linear_vel + robot_angular_vel + pos_gripper + rot_gripper 
        

        # For the actions
        n_thrusters = 5
        n_servos = 6 # 4 servos + 2 finger servos
        n_total_actions = n_thrusters + n_servos

        # Thruster limits
        thruster_low = np.full((n_thrusters,), -10.0, dtype=np.float32)
        thruster_high = np.full((n_thrusters,), 10.0, dtype=np.float32)

        # Servo limits
        servo_low = np.full((n_servos,), -1.0, dtype=np.float32)
        servo_high = np.full((n_servos,), 1.0, dtype=np.float32)   

        low = np.concatenate([thruster_low, servo_low])
        high = np.concatenate([thruster_high, servo_high])      

        # Observation includes the last action, (n_total_obs + n_total_actions)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(n_total_obs + n_total_actions,), dtype=np.float32)
        self.action_space = spaces.Box(low=low, high=high, shape=(n_total_actions,), dtype=np.float32)

        self.last_action_aplied = np.zeros(n_total_actions, dtype=np.float32)


    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi


    def build_reset_command(self):
        """
        Build a random RESET command for Ball and girona500.
        """
        ball_pos = [
            self.np_random.uniform(-7.0, 7.0), # x --> Pool length
            self.np_random.uniform(-3.7, 4.5), # y --> Pool width
            5.0                                # z --> Pool height
        ]
        ball_rot = [0.0, 0.0, 0.0]  # Rotation of the "Ball" robot

        girona_pos = [0.0, 0.0, 0.5] # Position of "girona500"
        girona_rot = [
            self.np_random.uniform(-np.pi, np.pi) , # girona500 rotation around x
            0.0, # girona500 rotation around y
            0.0  # girona500 rotation around z
        ]

        return [
            {
                "name": "girona500",
                "position": girona_pos,
                "rotation": girona_rot
            },
            {
                "name": "Ball",
                "position": ball_pos,
                "rotation": ball_rot
            }
        ]
        

    def reset(self, seed=None, options=None):
        """ 
        Reset the simulation, randomly reposition the "Ball" robot, and return an observation.
        """
        # Build the RESET command
        command = self.build_reset_command()

        # Send the command to Stonefish
        obs = self.send_command("RESET:" + json.dumps(command) + ";")

        super().reset(obs, seed=seed, options=options)
        
        self.step_counter = 0
        obs = self.get_observation()
        info = {}

        return obs, info

    
    def create_command(self, values):
        """
        Create a command dictionary from action values.
        """
        action = np.array(values).flatten()
        
        control_type = {
            "girona500/ThrusterSurgePort": "TORQUE",
            "girona500/ThrusterSurgeStarboard": "TORQUE",
            "girona500/ThrusterSway": "TORQUE",
            "girona500/ThrusterHeaveBow": "TORQUE",
            "girona500/ThrusterHeaveStern": "TORQUE",
            "girona500/Servo1": "VELOCITY",
            "girona500/Servo2": "VELOCITY",
            "girona500/Servo3": "VELOCITY",
            "girona500/Servo4": "VELOCITY",
            "girona500/FingerServo1": "VELOCITY",
            "girona500/FingerServo2": "VELOCITY"
        }

        # Create the command dictionary
        command = {}
        for name, val in zip(control_type.keys(), action):
            command[name] = {control_type[name]: float(val)}

        return command
            

    def step(self, action):
        """
        Makes one step: applying commands to the simulator and obtaining an observation.
        """
        self.step_counter += 1

        command = self.create_command(action)

        # Convert the command to a string (to send to Stonefish) and advance the simulation 'steps' times
        cmd_string = self.build_command(command)
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)

        obs = self.get_observation()
        reward = self.calculate_reward() 

        terminated = False 
        truncated = False

        if(reward == 0):
            # The gripper is already close enough (< 0.5 m) to the "Ball" robot
            terminated = True

        elif (self.step_counter * self.dt >= self.search_time):
            # Exceeded the allowed time to search for the "Ball" robot
            truncated = True

        info = {}

        self.last_action_aplied = np.array(action, dtype=np.float32).flatten()

        return obs, reward, terminated, truncated, info


    def safe_vector(self, key, subkey, n=3):
        """
        Return the first 'n' elements of the vector, otherwise return [nan] * n.
        """
        return self.state.get(key, {}).get(subkey, [np.nan] * n)[:n]


    def get_observation(self):
        """
        Returns an observation with:
        - Ball position (3)
        
        - girona500 position (3)
        - girona500 rotation (3)
        
        - girona500 linear velocity (3)
        - girona500 angular velocity (3)
        
        - girona500 gripper position (3)
        - girona500 gripper rotation (3)
        
        - Joint angles of the girona500 arm (n)
        
        - Last action (n+m) 
        """

        obs = []
        
        # Ball position
        obs += self.safe_vector("Ball", "position", 3)

        # girona500 position and rotation
        obs += self.safe_vector("girona500", "position", 3)
        obs += self.safe_vector("girona500", "rotation", 3)

        # girona500 linear and angular velocity
        obs += self.safe_vector("girona500/dynamics", "linear_velocity", 3)
        obs += self.safe_vector("girona500/dynamics", "angular_velocity", 3)
    
        # girona500 gripper position and rotation
        obs += self.safe_vector("girona500/OdoGripper", "position", 3)
        obs += self.safe_vector("girona500/OdoGripper", "rotation", 3)
        
        # Arm joint angles
        for name, value in self.state.items():
            if ("Servo" in name or "Finger" in name) and "angle" in value:
                angle = value["angle"]
                if angle is not None:
                    obs.append(self.normalize_angle(angle))

        # Last applied action
        obs += self.last_action_aplied.tolist()
                
        return np.array(obs, dtype=np.float32)


    def dist_gripper_object(self):
        """
        Calculate the distance between the gripper and the other robot ("Ball").
        """
        vec_xyz_ball = self.state['Ball']['position']
        vec_xyz_gripper = self.state['girona500/OdoGripper']['position']

        dist = np.linalg.norm(np.array(vec_xyz_gripper) - np.array(vec_xyz_ball))
        return dist
    

    def calculate_reward(self):
        """
        Calculate the reward based on the distance between the gripper and the other robot ("Ball").
        """
        reward = 0
        actual_dist = self.dist_gripper_object()
        if(self.ok_dist_gripper_object < actual_dist):
            reward = actual_dist *-1

        return reward
