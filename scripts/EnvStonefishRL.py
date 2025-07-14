import zmq
import json
import struct
import random
import gymnasium as gym
from gymnasium import spaces


class EnvStonefishRL(gym.Env):

    def __init__(self):
        super().__init__() # Crida al constructor de la classe gym (gymnasium)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5555")

        self.state = {} # Diccionari amb tota la informació 

        # Tindra els noms dels objectes de l'escena calsssificats segons el tipus q siguin
        self.robots = []
        self.actuators = []
        self.sensors = []

        # S'emplenaran segons les accions o observacions que vulgem fer
        self.action_space = spaces.Dict({})       # A la classe filla ja es definiran i agafaran un tipus concret
        self.observation_space = spaces.Dict({})  # Encara no sabem quants actuadors o sensors tindran. 


    # El que es mostra a la terminal de Python ja es amb el canvi a 'NaN' fet
    def _replace_null_with_nan(self, data):
        """
        Canvia els 'null' del diccionari per 'NaN'
        Pot evitar problemes amb Python pq 'None' no es tractat com un numero, però 'NaN' si
        """

        if data is None:
            return float('nan')
        
        if isinstance(data, dict):
            return {k: self._replace_null_with_nan(v) for k, v in data.items()}
        
        # Si no es dict ni list ni None, retorna tal cual la dada amb el tipus q era
        return data
    

    def _process_and_update_state(self, msg):
        """
        Rep les observacions (el string de JSON del simulador), les processa i 
        actualitza l'estat a l'ultima observació (la més recent) que s'ha fet del mapa
        """
        obs_dict = json.loads(msg)
        
        # Converteix els 'None' a 'NaN'
        obs_dict = self._replace_null_with_nan(obs_dict)

        # Actualitza l'estat intern
        self.state = obs_dict

        # Actualitza les llistes de robots, actuadors i sensors
        self.robots, self.actuators, self.sensors = self.list_objects_by_type()
        
        return self.state


    def build_command(self, command_dict):
        """
        Construeix un string CMD a partir d'un diccionari de commands
        """
        parts = []
        for actuator, params in command_dict.items():
            for param_name, value in params.items():
                parts.append(f"{actuator}:{param_name}:{value}")
        return "CMD:" + ";".join(parts) + ";OBS:"

    
    def send_command(self, message):
        """
        Envia una comanda al simulador StonefishRL
        """
        print(f"[CONN] Enviant comanda: {message}")
        self.socket.send_string(message)

        # Espera rebre una resposta del simulador
        response = self.socket.recv_string()
        print(f"[CONN] Resposta rebuda de StonefishRL: {response}")
        return response


    def close(self):
        _ = self.send_command("EXIT")
        self.socket.close()
        self.context.term()
        print("[INFO] SIMULACIÓ ACABADA.")  


    # Cal el "*, seed=None, options=None)" ?
    def reset(self, seed=None, options=None):
        
        x = random.uniform(-5.0, 0.0)
        y = random.uniform(-5.0, 0.0)
        z = random.uniform(-5.0, 0.0)
        roll = random.uniform(-3.14, 3.14)
        pitch = random.uniform(-3.14, 3.14)
        yaw = random.uniform(-3.14, 3.14)

        obs = self.send_command("RESET:Acrobot;")
        print("[INFO] StonefishRL diu: ", obs)

        # Carregar valors de la posició on anirà el robot al fer el RESET
        valors = [-x,y,z, roll, pitch, yaw] 
        tipus_format = "f" * len(valors) # Posa a float tots els valors que hi ha al vector 'valors'
        novesPosicions = struct.pack(tipus_format, *valors)
        
        # Enviar posicions (floats)
        self.socket.send(novesPosicions)
        msg = self.socket.recv_string()
        
        self._process_and_update_state(msg)
        
        self.print_full_state()

        return self.state
    

    def list_objects_by_type(self):
        """
            Clasifica els objectes segons el tipus q siguin, es basa en el seu nom
            - Robots: nom sense '/'
            - Actuadors: noms que comencen per 'Servo'. Agafa el que hi ha després de '/'
            - Sensors: noms que cemencen per 'Encoder'. Agafa el que hi ha després de '/'
            Retorna llistes: (robots, actuators, sensors).
        """
        
        robots = []
        sensors = []
        actuators = []

        if isinstance(self.state, dict):

            for name in self.state.keys():
                
                if '/' in name:
                    base_name = name.split('/')[-1] # El '-1' és per: P.ex. "Acrobot/Servo" --> "Servo"
        
                    if base_name.startswith("Servo"):
                        actuators.append(name)
                    
                    elif base_name.startswith("Encoder"):
                        sensors.append(name)
                    
                    else:
                        pass # No es cap d'aquests dos tipus
                
                else: # Si no hi ha '/', és un robot
                    robots.append(name)
                
            print("Actuators: ", actuators)
            print("Sensors: ", sensors)
            print("Robots: ", robots)    
            
        return actuators, sensors, robots

    # FALTARA DEFINAR UNA FUNCIÓ STEP EN AQUESTA CLASSE TAMBÉ
    # Fara arribar la comanda que volen fer al step al simulador.
    #def step(self):
        """
        Envia les accions al simulador i rep les observacions
        - action: Array amb les accions que es vol fer a cada actuador
        """

        commands = "CMD:Acrobot/Servo:POSITION:2.9;Acrobot/Servo2:TORQUE:5.0;"
        obs_objects = "OBS:"
        message = commands + obs_objects

        for step in range(1000):
            if(step > 20000):
                commands = "CMD:Acrobot/Servo:POSITION:2.1;Acrobot/Servo2:VELOCITY:2.0;OBS:"
            
            print(f"[INFO] Step {step + 1}\n")

            msg = self.send_command(message)
        
        self._process_and_update_state(msg)  

    
    def print_full_state(self):
        """
        Mostra tots els valors que hi ha al diccionari
        """
        print("[DEBUG] Dins de 'self.state' hi ha :")
        for obj_name, attributes in self.state.items():
            print(f" - {obj_name}")
            for attr, value in attributes.items():
                print(f"    -> {attr}: {value}")
