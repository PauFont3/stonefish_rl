# AcrobotEnv — Reinforcement Learning Environment with Stonefish

This environment implements a Reinforcement Learning scenario to control an **Acrobot** robot using the **Gymnasium** library.  

The physics simulation runs in **Stonefish** (C++), and communication between Python and the simulator is managed by **ZeroMQ**.  
The agent’s goal is to learn how to swing the Acrobot so that the tip of its second link exceeds a target height, starting from an initial rest position.

---

## 1. Run the Environment

The Python scripts launch the Stonefish simulator with the corresponding scene automatically. You only need to **activate the Python environment** and run the desired script.

**Activate the virtual environment**  
From the project root:
```bash
source env_rl/bin/activate
```

**A. Train a New Agent**  
To train an agent from scratch using Stable-Baselines3 (PPO):
```bash
python scripts/acrobot/acro_training.py
```

**B. Evaluate a Trained Agent**  
To see how a trained agent behaves (make sure the model file exists):
```bash
python scripts/acrobot/evaluate_acro.py
```

**C. Manual Tests (DEBUG)**  
To test the environment without a model, sending random actions:
```bash
python scripts/acrobot/test_acrobot.py
```

#### Important Notes
- When `reset()` is called, the Acrobot may keep swinging for a while. This is normal and is caused by the residual velocity: the system tries to put the robot into a rest position by applying opposite forces to the current motion until it reaches a balanced position.

---


## 2. How It Works
1. The **C++ server (`StonefishRLTest`)** loads the scene file (`.xml`) and waits for a connection on port 5555.

2. The **Python client (`EnvStonefishRL.py`)** creates the connection and links it to the server using ZeroMQ.

3. When `env.step(...)` is called, Python sends a command to the Stonefish simulator, e.g. `CMD:TORQUE...`.

4. The simulator applies the torque, advances the simulation for several steps (`dt`) and reads the sensors data.

5. These sensor data is serialised to JSON and sent back to Python.

6. Python processes the JSON, updates the environment state, computes the reward, and checks whether the episode should end (`terminated` or `truncated`).

7. The `reset(...)` method sends a special `"RESET"` command that triggers the function `robot->Reset()` in C++ to restore the robot’s initial state.
