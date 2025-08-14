# G500TestEnv — Test Environment with Girona500

This environment (`G500TestEnv`) is intended for testing the Girona500 robot inside a virtual pool in Stonefish and **has no specific learning objective**.  
It's useful to verify that actuators, sensors and robots work correctly, that commands can be sent and observations received, and that steps can be performed as expected in the environment.

---

## Run the environment
Activate your virtual environment:
```bash
source env_rl/bin/activate
```
  
Run the test:
```bash
python scripts/tests_sensors_actuators/test.py
```

This will:
1. Automatically launch the Stonefish simulator with the test scene (`girona500_basic.scn`).  
   > The test reuses the RL `girona500_basic.scn` scene and was adapted to add and test sensors/actuators on an already working setup.
2. Instantiate the `G500TestEnv` environment.
3. Call `reset()` to place the `girona500` robot and the `"Ball"` object at random initial positions and rotations.
4. Enter a loop that sends random actions and calls `step()` continuously.

---

## Environment usage

Use this environment to:
- Verify that sensors and actuators work correctly after modifying the scene or the code.
- Adjust action and observation ranges.
- Check that the simulator responds correctly when new elements are added.
- Verify that the robot’s position and movements are coherent.
> This environment relies on the Stonefish simulator. For details about scenes, components, and configuration, see the official docs:
https://github.com/patrykcieslak/stonefish
