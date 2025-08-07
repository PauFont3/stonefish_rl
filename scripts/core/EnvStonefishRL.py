import zmq
import json
import gymnasium as gym


class EnvStonefishRL(gym.Env):

    def __init__(self, ip="tcp://localhost:5555"):
        super().__init__() # Crida al constructor de la classe gym (gymnasium)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(ip)

        self.state = {} # Diccionari amb tota la informació 


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
        #print(f"[CONN] Enviant comanda: {message}")
        self.socket.send_string(message)

        # Espera rebre una resposta del simulador
        response = self.socket.recv_string()
        #print(f"[CONN] Resposta rebuda de StonefishRL: {response}")
        return response
    

    def close(self):
        _ = self.send_command("EXIT")
        self.socket.close()
        self.context.term()
        print("[INFO] SIMULACIÓ ACABADA.")  


    def reset(self, obs, seed=None, options=None):
        self._process_and_update_state(obs)

        super().reset(seed=seed)

        return self.state
    

    def step(self, message, steps):
        """
        Envia les accions al simulador i rep les observacions
        """
        for i in range(steps):
            msg = self.send_command(message)

        # Processar l'ultim estat rebut
        self._process_and_update_state(msg)
        self.print_full_state()
        
    
    def print_full_state(self):
        """
        Mostra tots els valors que hi ha al diccionari
        """
        print("[DEBUG] Dins de 'self.state' hi ha :")
        for obj_name, attributes in self.state.items():
            print(f" - {obj_name}")
            for attr, value in attributes.items():
                print(f"    -> {attr}: {value}")
