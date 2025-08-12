
# StonefishRL — Add a New Scalar Sensor (not Vision)

## 1) Include the sensor library (if needed)
- In `StonefishRL.cpp` or `StonefishRL.h`, add the appropriate header for the sensor type.
- Example:
```cpp
#include <Stonefish/sensors/scalar/IMU.h>
```

## 2) Define the sensor in the scene XML
- Add the new sensor to your scene file (`.scn`).
  
## 3) Extend `InfoObject` (if needed)
- Add the new sensor fields in `InfoObject` in `StonefishRL.h`:
```cpp
struct InfoObject {
    // …existing fields
    // Add your new sensor data here
};
```

## 4) Verify Stonefish detects the sensor (StonefishRL.cpp)
- Temporally enable `PrintAll()` to confirm the sensor appears and which data channels it exposes.  
> [!TIP]
> If the actuator doesn´t appear, start by checking the **name** given in the XML and if the type exists in StonefishRL.

> [!WARNING]
> Keeping `PrintAll()` commented during normal runs can print faster than Python and flood the console.
  
## 5) Add it to the observation vector
- In `StonefishRL.cpp` in `GetStateScene()` , follow the same pattern used by existing sensors to gather the values you want to export to Python.  
- **Important:** remember to call `push_back` so the observations are appended as `InfoObject` entries.

## 6) Send the data to Python
- In `InfoObjectToJson(...)`, add the fields using `SafeFloat(...)` so unwanted fields become `NaN`.
- In `FillWithNanInfoObject(...)`, initialise the new sensor fields you plan to collect with `NaN`.

## 7) Confirm Python recieves the values
- In `EnvStonefishRL.py`, uncomment the line `self.print_full_state()` inside `step(self, message, steps)` to print on screen the values coming from the newly added sensor.  

---

---

# StonefishRL — Add a New Actuator

## 1) Include the actuator library (if needed)
- In `StonefishRL.cpp` or `StonefishRL.h`, add the appropriate header for the actuator type.  
- Example:
```cpp
#include <Stonefish/actuators/Servo.h>
```

## 2) Define the actuator in the scene XML
- Add the new actuator to your scene file (`.scn`).
- Make sure the actuator has a **unique name**.

## 3) Verify Stonefish detects the actuator (StonefishRL.cpp)
- The function `BuildScenario()` stores all the actuators from the scene in a map `actuators_`.
- In the function `SendObservations()` you can enable `PrintAll()` to see if the new actuator appears.
> [!NOTE]
> Use `PrintAll()` only for debugging, in normal runs it can spam so many C++ messages to the console.

## 4) Enable control of the new actuator in C++ 
- In `StonefishRL.cpp` in the function `ApplyCommands(...)`, there is a `switch` that handles different actuator types.  
- Define how to apply commands to the actuator. Choose the control modes (e.g. `VELOCITY`, `TORQUE`, `POSITION`, or any other new custom keyword).  
- If the actuator it's not already covered add a new `case`.  
Here's a template:
```cpp
// CUSTOM
case sf::ActuatorType::CUSTOM:
{
    sf::CustomActuator* act = dynamic_cast<sf::CustomActuator*>(actuator_ptr);
    if (!act) {
        std::cout << "[WARNING] Not converted " << actuator_name << " to CUSTOM.\n";
        break;
    }

    for (const auto& [action, value] : commands_[actuator_name]) {
        if (action == "MY_ACTION1")      act->DoSomething(value);
        else if (action == "MY_ACTION2") act->DoSomethingElse(value);
        else {
            std::cout << "[WARNING] Unknown command '" << action << "' for custom '" << actuator_name << "'\n";
        }
    }
    break;
}
```

## 5) Add actuator data to the observations (optional)  
- In `StonefishRL.cpp` in `GetStateScene()`, in the actuators loop, add your branch and fill the field with the actuators data (if not already included) in `InfoObject`.
```cpp
else if(actuator_ptr->getType() == sf::ActuatorType::MY_ACTUATOR) {
    MyActuator* act = dynamic_cast<MyActuator*>(actuator_ptr);
    if(!act) continue;
    obs.my_field = act->getSomething();   // Replace with the actual actuator functions
    state.observations.push_back(obs);
}
```
> Only include relevant fields. Also, if you add new fields to `InfoObject`, initialize them in `FillWithNanInfoObject(...)` and serialize them in `InfoObjectToJson(...)`.

## 6. Map the actuator in the Python environment
- In your `Env` you need to specify how actions correspond to the new actuator’s commands. This is done by mapping the actuator’s name to the command key you defined in C++. 
 - Implement or update the function `create_command(...)` to include the new actuator.
 - Make sure the key (`name`) matches the actuator’s name in the scene (including any robot prefix).
 - Example:
```python
  def create_command(self, values):
    action = np.array(values).flatten()
    control_type = {
        "MyRobot/MyNewActuator": "MY_ACTION1",
        "MyRobot/AnotherActuator": "TORQUE",
        ...
    }
    command = {}
    for (name, act_value) in zip(control_type.keys(), action):
        command[name] = { control_type[name]: float(act_value) }
    return command
```
> The `values` parameter (can be a list or array) contains the values for each actuator's command in the same order as the map keys in `control_type`.  
> The base class (EnvStonefishRL) will convert this to the proper string format when you call the function`build_command()`.  

## 7) Extend the action space in Gym
- Whenever a controllable actuators added to the environment, you must update the Gym environment’s action space to include it.
- In the environment's `__init__`:
  - Increase the number of total actions (e.g. `n_total_actions = previous_actions + n_new_actions`).
  - Define the range of valid values for that actuator (only if needed).
  - Also consider if the observation space needs to be expanded.

---

---

# StonefishRL — Create Commands to Control Actuators

Commands are messages sent from the Python environment to the Stonefish simulator (C++) to control actuators, reset the scene, or terminate the simulation.

## 1 Command message format (Pyhton to C++)

```
CMD:ActuatorName:ACTION_KEY:VALUE;OBS:
```
- `CMD:` - Indicates the beginning of a command (this tells the simulator we are sending actuator control commands).
- `ActuatorName:` - The name of the actuator as defined in the Stonefish scene (including any prefix like the robot name, e.g. `girona500/Servo1`).
- `ACTION_KEY:` - The keyword for the type of action or control mode (e.g. `POSITION`, `VELOCITY`, `TORQUE`, ...).
- `VALUE;` - Number representing the command value.
> `;` – Acts as a separator between multiple commands. You can send several actuator commands in one message by separating them with `;`.
- `OBS:` - Indicates the end of the command and the start of the observations (even if no specific observation requests are made, the format expects this `OBS:`).  

Example:
```
CMD:Acrobot/Servo2:TORQUE:1.5;OBS:

# You could also send multiple commands at once.
CMD:MyRobot/Thruster1:VELOCITY:0.8;MyRobot/Servo1:TORQUE:10;OBS:
```

## 2) Building command string in Python

You don't have to build these strings manually. In the Gym environment class, after defining the method `create_command()` (returns a dictionary of commands), the base class `EnvStonefishRL` has a method `build_command(command_dict)` to construct the string. 

- The create_command method:
  - **Keys** are actuator names (matching those in the simulator).
  - **Values** are dictionaries mapping an action keyword to it's value.
  For example:
  ```python
  {
    "MyRobot/Thruster1": {"VELOCITY": 0.8},  
    "MyRobot/Servo1": {"TORQUE": 10.0}  
  }
  ```
  
- The base method `build_command` will iterate through this and produce:
```
CMD:MyRobot/Thruster1:VELOCITY:0.8;MyRobot/Servo1:TORQUE:10.0;OBS:  
```

- To **add a new command type**: : ensure you use a unique keyword in the dictionary (as the `ACTION_KEY`). Then confirm the C++ `ApplyCommands` function knows how to handle that keyword for the given actuator. If it’s an existing actuator but new mode, you may need to extend the C++ handling for that actuator to interpret the new key.

## 3) Special commands: RESET and EXIT

In addition to the `CMD:`, there two other commands:
- `RESET:` - This command resets the environment with new specified positions/orientations for robots.

```
RESET:[{"name":"MyRobot1","position":[0,0,0.5],"rotation":[0,0,0]},
       {"name":"MyRobot2","position":[-2,1.3,5.0],"rotation":[0,0,0]}]
```

You can find an example in the `G500Env.py` with the functions `build_reset_command()` and `reset(...)`

- `EXIT` - This command tells the simulator to end. The base environment’s method `close()` sends all the necesary to shut it down.  

> [!NOTE]  
> These three commands are handled in C++ by the `ReceiveInstructions()` function. Which checks the prefix of the command:  
> - If it starts with `"CMD:"`, the simulator will parse it as one or multiple actuator commands.  
> - If it starts with `"RESET:"`, the simulator will parse the JSON message and reset the scenario accordingly.  
> - If its `"EXIT"`, the simulator breaks out of its loop and closes (no observation response expected).  


## 4) How the simulator processes commands (JUST INFO)

- The Python sends a command (via `socket.send_string` in ZMQ) and the C++ recieves it. 

- The method `ReceiveInstructions()` in C++ will distinguish the type of command.

- For a `CMD` command: it will call the method `ParseCommandsAndObservations(...)` to decodify the string into individual actuator commands.
  
- The results are stored in a map (`commands_`) where each actuator name maps to a pair of (action_key, value).

- After parsing, the simulator applies each command.
  - For each actuator name it finds the corresponding actuator (checking the name in `actutors_` map built in the scenario loading time).
  - Then it checks the action key and applies the method on the actuator. (This is where your new code from step 3 for the new actuator would run, if the name and key match).

- After applying all commands, the simulator advances the physics for a fixed number of steps (the number of steps is determined by the Python side).
  
- Collects all the sensor and actuaor readings into a new observation whcih is a string. This observation string is sent back to the Python side.
  
- Finally, the Python environment receives the observation string (via `socket.recv_string()` in the method `send_command`) and processes it (using `_process_and_update_state` to update the `env.state` dictionary).

---

---

# StonefishRL — Add a New Robot

## 1) Include the robot library (if needed)
- In `StonefishRL.cpp` or `StonefishRL.h`, add the appropriate header.
```cpp
#include <Stonefish/core/Robot.h>
```

## 2) Define the robot in the XML  
- Add the new robot to your scene file (`.scn`).
- Set an initial position and rotation.
- Give a unique name to the robot.
> [!NOTE]
> The **exact** names in the scene will be used in C++ and Python.

## 3) Add the actuators and sensors (optional)
- Inside the robot block, declare the actuators you will control (e.g. Thruster1, Servo1, ...).
- Add at least one sensor for the observations (e.g. Odometry, IMU, encoders, ...).

## 4) Verify the robot has been detcted
- Temporally enable the method `PrintAll()` to confirm the robot. actuators and sensors are listed.

## 5) Reset pose form Python (Optional)  
You can reposition the robot with a `RESET:` command.
