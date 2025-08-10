
# StonefishRL — Add a New Scalar Sensor (not Vision)
## 1) Include the sensor library (if needed)
- In `StonefishRL.cpp` or `.h`, add the appropriate header for the sensor type.
- Example:
```cpp
#include <Stonefish/sensors/scalar/IMU.h>
```

## 2) Define the sensor in the scene XML
- Add the new sensor to your scene file (`.scn`).
  
## 3) Extend `InfoObject` (StonefishRL.h)
- Add the new sensor fields to:
```cpp
struct InfoObject {
    // …existing fields
    // Add your new sensor data here
};
```

## 4) Verify Stonefish detects the sensor (StonefishRL.cpp)
- Use `ProvaMostrarTot()` to print in the terminal whether the sensor appears and which data channels it exposes.

## 5) Add it to the observation vector
- In `GetStateScene()`, follow the same pattern used by existing sensors to gather the values you want to export to Python.  
- **Important:** remember to call `push_back` so the observations are appended as `InfoObject` entries.

## 6) Send the data to Python
- In `InfoObjectToJson(...)`, add the fields using `SafeFloat(...)` so unwanted fields become `NaN`.
- In `FillWithNanInfoObject(...)`, initialise the new sensor fields you plan to collect with `NaN`.

## 7) Confirm Python recieves the values
- In `EnvStonefishRL.py`, uncomment the line `self.print_full_state()` inside `step(self, message, steps)` to print on screen the values coming from the newly added sensor.

---
