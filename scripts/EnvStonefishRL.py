import zmq
import json
import struct
import gymnasium as gym
from gymnasium import spaces


class EnvStonefishRL(gym.Env):

    def __init__(self):
        super().__init__() # Crida al constructor de la classe gym (gymnasium)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5555")

        self.state = {} # Diccionari amb tota la informació 

        # Contindrà els noms dels objectes de l'escena calsssificats segons el tipus q siguin
        self.robots = []
        self.actuators = []
        self.sensors = []

        # S'emplenaran segons les accions o observacions que vulgem fer
        self.action_space = spaces.Dict({}) # Diccionari amb totes les accions que es poden fer
        self.observation_space = spaces.Dict({}) # Diccionari amb totes les observacions que es poden fer


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
        if isinstance(data, list):
            return [self._replace_null_with_nan(v) for v in data]
        
        # Si no es dict ni list ni None, retorna tal cual la dada amb el tipus q era
        return data


    def close(self):
        self.socket.send_string("EXIT")
        _ = self.socket.recv_string() # Espera rebre alguna confirmacio que dient q el simulador s'ha tancat
        self.socket.close()
        self.context.term()
        print("[Python] SIMULACIÓ ACABADA.")  


    def reset(self):
        # Enviar string amb la comanda (instrucció) 
        print("[Python] Enviant Reset a StonefishRL")
        self.socket.send_string("RESET:Acrobot;")

        # Carregar valors de la posició on anirà el robot al fer el RESET
        valors = [-3.0, -4.0, -5.0, 2.0, 2.0, 2.0] # Podem afegir les rotacions
        tipus_format = "f" * len(valors) # Posa a float tots els valors que hi ha al vector 'valors'
        novesPosicions = struct.pack(tipus_format, *valors)
        
        # Rebre de confirmació de C++ indicant que ja esta preparat per rebre les posicions
        obs = self.socket.recv_string()
        print("[Python] StonefishRL diu: ", obs)
        
        # Enviar posicions (floats)
        self.socket.send(novesPosicions)

        # Rebre confirmació de C++
        resposta = self.socket.recv_string()
        print("[Python] StonefishRL ha dit:", resposta)
        # Retornar l'estat inicial del robot
    

    def _recieve_obs(self):
        msg = self.socket.recv_string()
        obs_dict = json.loads(msg)

        # Converitr els None a NaN, per evitar possibles problemes amb Python.
        # Pq 'None' no es un numero, en canvi 'NaN' el poden interpretar com 
        # un numero sense un valor definit.
        for obj_name, values in obs_dict.items():
            for k, v in values.items():
                
                # Crea una nova llista on cada 'None' es converteix a 'NaN'
                if isinstance(v, list):
                    new_list = []
                    for item in v:
                        if item is None:
                            new_list.append(float('nan'))
                        else:
                            new_list.append(item)
                    obs_dict[obj_name][k] = new_list
                
                # Si no es una llista, però és un 'None' el converteix a 'NaN'
                elif v is None:
                    obs_dict[obj_name][k] = float('nan')
                
                # SI no és ni llista ni 'None', el deixa tal cual esta
                else:
                    obs_dict[obj_name][k] = v                   

        self.state = obs_dict
        return obs_dict


    def list_objects_by_type(self):
        """
            Clasifica els objectes segons el tipus q siguin tipo, es basa en el seu nom
            - Robots: nom sense '/'
            - Actuadors: noms que comencen per 'Servo'. Agafa el que hi ha després de '/'
            - Sensors: noms que cemencen per 'Encoder'. Agafa el que hi ha després de '/'
            Retorna llistes: (robots, actuators, sensors).
        """
    
        actuators = []
        sensors = []
        robots = []
    
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
            
        return actuators, sensors, robots


    def _get_observation(self): 
        """
        Retorna un diccionari amb les observacions de tots els objectes de l'escena
        """

        obs = self._recieve_obs()
        actuators, sensors, robots = self.list_objects_by_type()

        print("[Python] Actuators: ", actuators)
        print("[Python] Sensors: ", sensors)
        print("[Python] Robots: ", robots)

        return obs  


    def step(self):

        """
        Envia les accions al simulador i rep les observacions
        - action: Array amb les accions que es vol fer a cada actuador
        """

        commands = "CMD:Acrobot/Servo:POSITION:2.9;Acrobot/Servo2:TORQUE:5.0;"
        obs_objects = "OBS:"
        message = commands + obs_objects

        for step in range(10000):
            if(step > 20000):
                commands = "CMD:Acrobot/Servo:POSITION:2.1;Acrobot/Servo2:VELOCITY:2.0;OBS:"
            
            print(f"[Python] Step {step + 1}\n")

            print("[Python] Enviant Comandes a StonefishRL: ", message)
            self.socket.send_string(message)
            obs = self._recieve_obs()
            print("[Python] Observacions rebudes a Python: ", obs)



stop = False
env = EnvStonefishRL()

while not stop:
    print("\n--- MENU ---")
    print("1. Send ApplyCommands")
    print("2. Send Reset")
    print("3. Exit Simulation")
    option = input("Select an option: ")

    if option == "1": 
        # Exemple Comanda: "CMD:Robot1/Servo:POSITION:3.5;Robot2/Servo:VELOCITY:2.0;OBS:Robot1,Robot2"
        #                  "CMD:Robot1/Servo:POSITION:3.5;OBS:" --> Mostra les observacions de tots els objectes de l'escena
        env.step()
    
    elif option == "2": 
        env.reset()

    elif option == "3":
        stop = True
        env.close()
        print("[Python] S'ha tancat la simulació.")