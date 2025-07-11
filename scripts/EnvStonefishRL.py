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

    
    def send_command(self, message):
        """
        Envia una comanda al simulador StonefishRL
        """
        print(f"[INFO] Enviant comanda: {message}")
        self.socket.send_string(message)

        # Espera rebre una resposta del simulador
        response = self.socket.recv_string()
        print(f"Resposta rebuda de StonefishRL: {response}")
        return response


    def close(self):
        _ = self.send_command("EXIT")
        self.socket.close()
        self.context.term()
        print("[INFO] SIMULACIÓ ACABADA.")  


    def reset(self, *, seed=None, options=None):
        
        obs = self.send_command("RESET:Acrobot;")
        print("[INFO] StonefishRL diu: ", obs)

        # Carregar valors de la posició on anirà el robot al fer el RESET
        valors = [-3.0, -4.0, -5.0, 2.0, 2.0, 2.0] # Podem afegir les rotacions
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
        print("[DEBUG] Dins de 'self.state' hi ha :")
        for obj_name, attributes in self.state.items():
            print(f" - {obj_name}")
            for attr, value in attributes.items():
                print(f"    -> {attr}: {value}")



#if __name__ == "__main__":
 #   stop = False
 #   env = EnvStonefishRL()

 #   while not stop:
  #      print("\n--- MENU ---")
  #      print("1. Send ApplyCommands")
  #      print("2. Send Reset")
  #      print("3. Exit Simulation")
  #      print("4. Print Dictionary")
  #      print("5. Print Objects by type")

  #      option = input("Select an option: ")

  #      if option == "1": 
  #          # Exemple Comanda: "CMD:Robot1/Servo:POSITION:3.5;Robot2/Servo:VELOCITY:2.0;OBS:Robot1,Robot2"
  #          #                  "CMD:Robot1/Servo:POSITION:3.5;OBS:" --> Mostra les observacions de tots els objectes de l'escena
  #          env.step()
  #      
  #      elif option == "2":

  #          env.reset()
  #          env.print_full_state()

  #      elif option == "3":
  #          stop = True
  #          env.close()
  #          print("[INFO] S'ha tancat la simulació.")

  #      elif option == "4":
  #          env.print_full_state()

  #      elif option == "5":
  #          env.list_objects_by_type()