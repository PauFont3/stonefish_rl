import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")


while True:
    print("\n--- MENU ---")
    print("1. Send ApplyCommands")
    print("2. Send Reset")
    option = input("Select an option: ")

    if option == "1": 

        commands = "CMD:Robot/Servo:1000.75;Robot/Servo2:2000.75"

        for step in range(50):
            
            if(step == 2000):
                commands = "CMD:Robot/Servo:0.0;Robot/Servo2:0.0"
            
            print(f"[Python] Step {step + 1}\n")

            print("[Python] Enviant Comandes a StonefishRL: ", commands)
            socket.send_string(commands)
            obs = socket.recv_string()
            print("[Python] Observacions rebudes a Python: ", obs)

    elif option == "2":

        print("[Python] Enviant Reset a StonefishRL")
        socket.send_string("RESET")
        obs = socket.recv_string()
        print("[Python] Observacions rebudes a Python: ", obs)        
    