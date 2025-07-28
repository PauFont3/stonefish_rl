from EnvStonefishRL import EnvStonefishRL
import numpy as np
import struct
from gymnasium import spaces
import json


class G500Env(EnvStonefishRL):
    def __init__(self, ip="tcp://localhost:5555", search_time=20):
        super().__init__(ip)

        self.step_counter = 0
        
        self.ok_dist_gripper_object = 0.5 # Distancia acceptable entre objecte i gripper
        self.search_time = search_time # Temps que volem que el robot estigui explorant

        self.sim_stonefishRL = 0.001 # Delta Time de Stonefish.
        self.dt = 0.2 # Delta Time de l'entorn que aplica el Reinforcement Learning.

        self.last_action = []

        n_ball_position = 3
        n_position = 3
        n_rotation = 3
        n_joints = 6  # 4 servos + 2 finger servos
        n_total_obs = n_ball_position + n_position + n_rotation + n_joints
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(n_total_obs,), dtype=np.float32)
        
        n_thrusters = 5
        n_servos = 6
        n_total_actions = n_thrusters + n_servos
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(n_total_actions,), dtype=np.float32)


    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


    def reset(self, seed=None, options=None):
        """ 
        Reinicia la simulació, recoloca aleatoriament el robot ("Ball") i retorna una observació
        """
        if options is None:
            options = {}

        low = -7.2
        high = 7.2 

        # Genera una posició i orientació aleatoria pel robot ("Ball")
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

        super().reset(seed=seed, options=options)
        
        self.step_counter = 0
        obs = self.get_observation()
        info = {}

        return obs, info

    
    def create_command(self, value):
        command = {
            "girona500/ThrusterSurgePort": {
                "TORQUE": -5.0
            },
            #"girona500/ThrusterHeaveStern": {
            #    "TORQUE": value
            #}
            #,
            # "girona500/ThrusterHeaveBow": {
            #    "TORQUE": value
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

        return command
    

    def step(self, action):
        """
        Executa un step a la simulació aplicant les comandes escrites, també obté una observació
        """
        self.step_counter += 1

        # Canviar els valors segons el q vulguem
        torque = [-50.0, 0.0, 50.0]
        tau = torque[action]

        command = self.create_command(tau)

        # Converteix la comanda en string (per poder enviar a Stonefish) i avança la simulacio 'steps' vegades
        cmd_string = self.build_command(command)
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)

        obs = self.get_observation()
        reward = self.calculate_reward() 

        print("Dist: ", reward *-1) # *-1 pq el reward és la distancia però en negatiu, cal canviar a posotiu per aconseguir la distancia 

        terminated = False 
        truncated = False

        if(reward == 0):
            # El gripper ja està suficientment aprop (>0.5) del robot ("Ball")
            terminated = True

        elif (self.step_counter * self.dt >= self.search_time):
            # Superat el temps que tenia per buscar l'altre robot ("Ball") 
            truncated = True

        info = {}

        return obs, reward, terminated, truncated, info


    def safe_vector(self, key, subkey, n=3):
        """
        Retorna les 3 primeres posicions del vector
        """
        if key in self.state:
            if subkey in self.state[key]:
                return self.state[key][subkey][:n]
            else:
                return [np.nan] * n
        else:
            return [np.nan] * n
        

    def get_observation(self):
        """
        Retorna una observació amb:
        - Posició de la bola (3)
        - Posició del girona500 (3)
        - Rotació del girona500 (3)
        - Ángles de les juntes del girona500 (n)
        - Ultima acció (n) (FALTA IMPLEMENTAR)
        """
        obs = []
        
        #  Posició de la bola
        obs += self.safe_vector("Ball", "position", 3)

        # Posicio i rotació del girona500
        obs+= self.safe_vector("girona500", "position", 3)
        obs += self.safe_vector("girona500", "rotation", 3)

        # Angles de les joints del braç
        for name, value in self.state.items():
            if ("Servo" in name or "Finger" in name) and "angle" in value:
                angle = value["angle"]
                if angle is not None:
                    obs.append(self.normalize_angle(angle))


        return np.array(obs, dtype=np.float32)


    def dist_gripper_object(self):
        """
        Calcula la distància entre el gripper i l'altre robot ("Ball")
        """
        vec_xyz_ball = self.state['Ball']['position']
        vec_xyz_gripper = self.state['girona500/OdoGripper']['position']
        #rot_xyz_gripper = self.state['girona500/OdoGripper']['rotation']

        dist = np.linalg.norm(np.array(vec_xyz_gripper) - np.array(vec_xyz_ball))
        return dist
    

    def calculate_reward(self):
        """
        Calcula el reward segons la distància entre el gripper i l'altre robot ("Ball")
        """
        reward = 0
        actual_dist = self.dist_gripper_object()
        if(self.ok_dist_gripper_object < actual_dist):
            reward = actual_dist *-1

        return reward
    

    