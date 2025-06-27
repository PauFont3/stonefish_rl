import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")


while True:
    print("\n--- MENU ---")
    print("1. Send ApplyCommands")
    print("2. Send Reset")
    print("3. Exit Simulation")
    option = input("Select an option: ")

    if option == "1": 

        commands = "Acrobot/Servo:POSITION:1.0;Acrobot/Servo2:POSITION:2.5"

        for step in range(1000000):
            
            if(step > 2000):
                commands = "Acrobot/Servo:VELOCITY:0.0;Acrobot/Servo2:VELOCITY:0.0"
            
            print(f"[Python] Step {step + 1}\n")

            print("[Python] Enviant Comandes a StonefishRL: ", commands)
            socket.send_string(commands)
            obs = socket.recv_string()
            print("[Python] Observacions rebudes a Python: ", obs)

    elif option == "2":

        print("[Python] Enviant Reset a StonefishRL")
        socket.send_string("RESET")
        obs = socket.recv_string()
        print("[Python] RESET DONE. WHAT NEXT?.")  


    elif option == "3":

        print("[Python] Enviant EXIT a StonefishRL")
        socket.send_string("EXIT")
        obs = socket.recv_string()
        print("[Python] SIMULACIÓ ACABADA.")  