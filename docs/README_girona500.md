# G500Env — Reinforcement Learning Environment with the Girona500 Robot

This environment implements a Reinforcement Learning scenario to control the `Girona500` robot and the `ECA5eMicro` manipulator arm inside a virtual pool using **Gymnasium**.  
The virtual pool matches the dimensions of the real CIRS pool.

---

## Prerequisites
Before you start, make sure you have the following installed:
- **Stonefish Simulator (v1.5.0 or higher)** — installed and working
- **Python 3.10** and `pip`
- **Virtual environment** with the project dependencies (`requirements.txt`)
- **CMake** and a **C++17** compiler
- **ZeroMQ** with bindings for C++ and Python (`pyzmq`, `cppzmq`)
- **Stable-Baselines3** (for training)

---


## Run the environment
The Python scripts automatically launch the Stonefish simulator with the appropriate scene. Just run the script you want with your Python environment activated.

**A. Train a New Agent**  
Train a PPO agent to control the Girona500 using Stable-Baselines3. The best model will be saved to `logs/best_model.zip`.
```bash
python scripts/g500/G500_training_ppo.py
```

**B. Avaluar un Agent Entrenat**
Carrega el model logs/best_model.zip i mostra el seu rendiment. Si el gripper s’aconsegueix acostar a **<= 0.5m** de la bola, l’episodi es considerarà assolit i `terminated = True`.
```bash 
python scripts/g500/evaluate_g500.py
```

**C. Manual Tests (DEBUG)**  
Test the environment without a model by sending random actions to the `girona500` robot.
```bash
python scripts/g500/test_G500.py
```

---

## How it works internally
1. Python (`G500Env.py`) creates a ZeroMQ connection to the C++ server (`StonefishRLTest`).
2. When `env.step(action)` is called, Python sends commands with the specific velocities for each thruster and/or servo.
3. C++ applies the commands, advances the simulation, and returns a JSON payload with observations.
4. Python parses this JSON, computes the reward, and determines whether the episode should end (`terminated` or `truncated`).



## Observations and actions
> The number in between parentheses indicates the length of the corresponding vector.

An observation (`obs`) is a vector of **38** real values:
- Ball position (3)
- Girona500 position and rotation (6)
- Girona500 linear and angular velocity (6)
- Gripper position and rotation (6)
- Joint angles (6)
- Last action applied (11)

Actions are a vector of **11** real values:
- 5 thrusters (TORQUE): range `[-10.0, 10.0]`
- 6 servos (VELOCITY): range `[-1.0, 1.0]`

## Episode termination policy
- `terminated = True`: if the gripper is **≤ 0.5 metres** from the ball.
- `truncated = True`: if **30 seconds** pass (defined by `search_time`) without achieving the goal.

## Reward
```python
# Reward function:
if dist > 0.5:
    reward = -dist
else:
    reward = 0
```
Therefore, the closer the gripper is to the ball, the higher (less negative / zero) the reward.
