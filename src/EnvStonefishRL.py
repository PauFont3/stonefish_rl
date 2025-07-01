import zmq
import struct
#import gym

"""""
class EnvStonefishRL(gym.Env):

    def __init__(self):
    # OPCIO 1:
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect("tcp://localhost:5555")
    # OPCIO 2:    
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5555")
    # Faltara definir 'action_space' i 'observation_space' 
    

    def close():
        socket.send_string("EXIT")
        _ = socket.recv_string() # (OPCIONAL) Esperar a rebre alguna confirmació de que el simulador s'ha tancat
        socket.close()
        context.term()


    # Probablement fara falta passar algun paramatre a la funció "reset()" com un 'State' o algu 
    def reset():
        
        return 0
"""""

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")
stop = False


while not stop:
    print("\n--- MENU ---")
    print("1. Send ApplyCommands")
    print("2. Send Reset")
    print("3. Exit Simulation")
    option = input("Select an option: ")

    if option == "1": 

        commands = "CMD:Acrobot/Servo:POSITION:2.1;Acrobot/Servo2:TORQUE:5.0;"

        for step in range(10000):
            
            if(step > 20000):
                commands = "CMD:Acrobot/Servo:POSITION:2.1;Acrobot/Servo2:VELOCITY:2.0;"
            
            print(f"[Python] Step {step + 1}\n")

            print("[Python] Enviant Comandes a StonefishRL: ", commands)
            socket.send_string(commands)
            obs = socket.recv_string()
            print("[Python] Observacions rebudes a Python: ", obs)

    elif option == "2":
        
        # Enviar string amb la comanda (instrucció) 
        print("[Python] Enviant Reset a StonefishRL")
        socket.send_string("RESET:Acrobot;")

        # Carregar valors de la posiicó on anirà el robot al fer el RESET
        valors = [-3.0, -4.0, -5.0, 2.0, 2.0, 2.0] # Podem arribar a afegir les rotacions
        tipus_format = "f" * len(valors) # Posa a float tots els valors que hi ha al vector 'valors'
        novesPosicions = struct.pack(tipus_format, *valors)
        
        # Rebre de confirmació de C++ indicant que ja esta preparat per rebre les posicions
        obs = socket.recv_string()
        print("[Python] StonefishRL diu: ", obs)
        
        # Enviar posicions (floats)
        socket.send(novesPosicions)
 
        # Rebre confirmació de C++
        resposta = socket.recv_string()
        print("[Python] RESET DONE. StonefishRL ha dit: ", resposta)

    elif option == "3":

        print("[Python] Enviant EXIT a StonefishRL")
        socket.send_string("EXIT")
        obs = socket.recv_string()
        print("[Python] SIMULACIÓ ACABADA.")  
        stop = True