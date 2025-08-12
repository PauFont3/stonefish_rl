import zmq
import json
import gymnasium as gym


class EnvStonefishRL(gym.Env):

    def __init__(self, ip="tcp://localhost:5555"):
        super().__init__() # Call Gym's (Gymnasium) base constructor
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(ip)

        self.state = {} # Dictionary with all the info


    # El que es mostra a la terminal de Python ja es amb el canvi a 'NaN' fet
    def _replace_null_with_nan(self, data):
        """
        Replace JSON 'null' values in the dictionary with 'NaN'.
        This can avoid issues in Python because 'None' is not treated as a number,
        while 'NaN' yes.
        Note: What you see printed in the Python terminal already has 'NaN' applied.
        """

        if data is None:
            return float('nan')
        
        if isinstance(data, dict):
            return {k: self._replace_null_with_nan(v) for k, v in data.items()}
        
        # If it's not a dict, list, or None, it returns the value (preserving its type)
        return data
    

    def _process_and_update_state(self, msg):
        """

        Recieves the observations (the simulator's JSON string), processes it,
        and updates the internal state to the latest observation.
        """
        obs_dict = json.loads(msg)
        
        # Convert 'None' to 'NaN'
        obs_dict = self._replace_null_with_nan(obs_dict)

        # Update internal state
        self.state = obs_dict

        return self.state


    def build_command(self, command_dict):
        """
        Build a CMD string from a command dictionary.
        """
        parts = []
        for actuator, params in command_dict.items():
            for param_name, value in params.items():
                parts.append(f"{actuator}:{param_name}:{value}")
        return "CMD:" + ";".join(parts) + ";OBS:"

    
    def send_command(self, message):
        """
        Send a command to the StonefishRL simulator and wait for a response.
        """
        #print(f"[CONN] Enviant comanda: {message}")
        self.socket.send_string(message)

        # Wait to receive a response from the simulator
        response = self.socket.recv_string()
        #print(f"[CONN] Resposta rebuda de StonefishRL: {response}")
        return response
    

    def close(self):
        _ = self.send_command("EXIT")
        self.socket.close()
        self.context.term()
        print("[INFO] SIMULATION ENDED.")  


    def reset(self, obs, seed=None, options=None):
        """
        Update internal state from the provided observation and calls Gym's reset.
        Note: This reset signature expects that 'obs' is the simulator's JSON string.
        """
        self._process_and_update_state(obs)

        super().reset(seed=seed)

        return self.state
    

    def step(self, message, steps):
        """
        Send actions to the simulator and recieve observations.
        """
        for i in range(steps):
            msg = self.send_command(message)

        # Process the last state received
        self._process_and_update_state(msg)
        self.print_full_state()
        
    
    def print_full_state(self):
        """
        Print all values contained in the dictionary.
        """
        print("[DEBUG] The dictionary 'self.state' contains:")
        for obj_name, attributes in self.state.items():
            print(f" - {obj_name}")
            for attr, value in attributes.items():
                print(f"    -> {attr}: {value}")
