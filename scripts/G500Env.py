from EnvStonefishRL import EnvStonefishRL
import numpy as np
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

        # Per les observacions
        ball_position = robot_position = robot_rotation = 3
        joints = 6  # 4 servos + 2 finger servos
        robot_linear_vel = robot_angular_vel = 1
        pos_gripper = rot_gripper = 3
        n_total_obs = ball_position + robot_position + robot_rotation + joints + robot_linear_vel + robot_angular_vel + pos_gripper + rot_gripper 
        
        # Per les accions
        n_thrusters = 5
        n_servos = 6 # 4 servos + 2 finger servos
        n_total_actions = n_thrusters + n_servos

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(n_total_obs + n_total_actions,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(n_total_actions,), dtype=np.float32)

        self.last_action_aplied = np.zeros(n_total_actions, dtype=np.float32)


    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


    def build_reset_command(self):

        ball_pos = [
            float(self.np_random.uniform(-7.0, 7.0)), # x --> Llargada piscina
            float(self.np_random.uniform(-3.7, 4.5)), # y --> Amplada piscina
            float(self.np_random.uniform(0.0, 4.0))   # z --> Altura piscina, "4.0" com a màxim per evitar que pugui reapareixer la bola just sobre el robot i marqui terminted  
        ]
        ball_rot = [0.0, 0.0, 0.0]  # Rotació del robot "Ball"

        girona_pos = [0.0, 0.0, 0.0] # Posició del girona500
        girona_rot = [
            float(self.np_random.uniform(-0.1, 0.1)), # rotació en x del girona500
            float(self.np_random.uniform(-0.1, 0.1)), # rotació en y del girona500
            float(self.np_random.uniform(-0.1, 0.1))  # rotació en z del girona500
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
        Reinicia la simulació, recoloca aleatoriament el robot ("Ball") i retorna una observació
        """
        #Construeix la comanda RESET
        command = self.build_reset_command()

        # Envia la comanda A Stonefish
        obs = self.send_command("RESET:" + json.dumps(command) + ";")

        super().reset(obs, seed=seed, options=options)
        
        self.step_counter = 0
        obs = self.get_observation()
        info = {}

        return obs, info

    
    def create_command(self, values):
        
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

        # Crea el diccionari de commands
        command = {}
        for name, val in zip(control_type.keys(), action):
            command[name] = {control_type[name]: float(val)}

        return command
            

    def step(self, action):
        """
        Fa una step a la simulació aplicant les comandes escrites, també obté una observació
        """
        self.step_counter += 1

        command = self.create_command(action)

        # Converteix la comanda en string (per poder enviar a Stonefish) i avança la simulacio 'steps' vegades
        cmd_string = self.build_command(command)
        steps = int(self.dt / self.sim_stonefishRL)
        super().step(cmd_string, steps)

        obs = self.get_observation()
        reward = self.calculate_reward() 

        print("Dist: ", reward *-1) # *-1 pq el reward és la distancia però en negatiu, cal canviar a positiu per aconseguir la distancia 

        terminated = False 
        truncated = False

        if(reward == 0):
            # El gripper ja està suficientment aprop (>0.5) del robot ("Ball")
            terminated = True

        elif (self.step_counter * self.dt >= self.search_time):
            # Superat el temps que tenia per buscar l'altre robot ("Ball") 
            truncated = True

        info = {}

        self.last_action_aplied = np.array(action, dtype=np.float32).flatten()

        return obs, reward, terminated, truncated, info


    def safe_vector(self, key, subkey, n=3):
        """
        Retorna les 3 primeres posicions del vector, sino retorna [nan]*n
        """
        return self.state.get(key, {}).get(subkey, [np.nan] * n)[:n]


    def get_observation(self):
        """
        Retorna una observació amb:
        - Posició de la bola (3)
        - Posició del girona500 (3)
        - Rotació del girona500 (3)
        - Angles de les joints del girona500 (n)
        - Velocitat lineal del girona500 (1)
        - Velocitat angular del girona500 (1)
        - Posició del gripper del girona500 (3)
        - Rotació del gripper del girona500 (3)
        - Ultima acció (n+m) 
        """
        obs = []
        
        # Posició de la bola
        obs += self.safe_vector("Ball", "position", 3)

        # Posicio i rotació del girona500
        obs += self.safe_vector("girona500", "position", 3)
        obs += self.safe_vector("girona500", "rotation", 3)

        # Angles de les joints del braç
        for name, value in self.state.items():
            if ("Servo" in name or "Finger" in name) and "angle" in value:
                angle = value["angle"]
                if angle is not None:
                    obs.append(self.normalize_angle(angle))
        
        # Velocitat lineal i angular del girona500
        obs.append(self.state.get("girona500/dynamics", {}).get("linear_velocity", float("nan")))
        obs.append(self.state.get("girona500/dynamics", {}).get("angular_velocity", float("nan")))

        # Posicio i rotació del braç del girona500
        obs += self.safe_vector("girona500/OdoGripper", "position", 3)
        obs += self.safe_vector("girona500/OdoGripper", "rotation", 3)

        # Afegim l'última acció aplicada
        obs += self.last_action_aplied.tolist()
 
        return np.array(obs, dtype=np.float32)


    def dist_gripper_object(self):
        """
        Calcula la distància entre el gripper i l'altre robot ("Ball")
        """
        vec_xyz_ball = self.state['Ball']['position']
        vec_xyz_gripper = self.state['girona500/OdoGripper']['position']

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