# G500TestEnv — Test Environment with Girona500

This environment (`G500TestEnv`) is intended for testing the Girona500 robot inside a virtual pool in Stonefish and **has no specific learning objective**.  
It is useful to verify that actuators, sensors and robots work correctly, that commands can be sent and observations received, and that steps can be performed in the environment.

---

## Run the environment
Make sure your virtual environment is activated:
```bash
source env_rl/bin/activate
```

**Run the test:**
```bash
python scripts/tests_sensors_actuadors/test.py
```

This will:
1. Automatically launch the Stonefish simulator with the test scene (`g500_basic.scn`).  
   > The scene `g500_basic.scn` uses the same name as the RL Girona500 scene because the original Girona500 scene was reused and adapted to add and test sensors/actuators on an already working setup.
2. Instantiate the `G500TestEnv` environment.
3. Call `reset()` to place the `girona500` robot and the `"Ball"` object at random initial positions and rotations.
4. Enter a loop that continuously sends random actions and calls `step()`.

---

## Environment usage

This environment is meant to:
- Verify that sensors and actuators work correctly after modifying the scene or the code.
- Adjust action and observation ranges.
- Check whether the simulator responds correctly to new added elements.
- Verify that the robot’s position and movements are coherent.
