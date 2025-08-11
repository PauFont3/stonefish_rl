
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
> Here's a template:
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

