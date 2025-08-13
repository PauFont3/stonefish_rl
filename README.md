# Reinforcement Learning Environments with Stonefish

This repository contains several simulation environments based on **Stonefish** and controlled via **Python**.

## Available Environments
- [AcrobotEnv](./docs/README_acrobot.md) – Control a two-link pendulum to reach a target height.
- [G500Env](./docs/README_girona500.md) – Girona500 robot with a gripper to approach a target object (Ball)
- [G500TestEnv](./docs/README_tests.md) – Testing environment for sensors and actuators
- [Installation](./docs/README_installation.md) – Setup Stonefish, build the C++ server, create the Python env and run scenes.
- [Manual](./docs/README_manual.md) – Developer guide: commands (CMD/RESET/EXIT), adding sensors/actuators, creating robots, create an env structure.

## Technologies
- **Simulator**: [Stonefish](https://github.com/patrykcieslak/stonefish)
- **Reinforcement Learning**: Gymnasium + stable-baselines3
- **Comunication**: ZeroMQ (pyzmq + cppzmq)

## Project Structure   

```
stonefish_rl/
├── include/                         # C++ headers
│   └── StonefishRL.h
├── logs/                            # PPO models and training evaluation files
│   ├── best_model.zip
│   └── evaluations.npz
├── requirements.txt                 # Python dependencies
├── Resources/                       # Scenes, models and textures for Stonefish 
│   ├── acrobot/
│   │   └── acrobot_scene.xml
│   ├── minimal/
│   │   └── minimal_scene.xml
│   ├── tests_sensors_actuators/
|   |   ├── data/
|   |   |    └── ...
|   |   └── scenarios/
|   |        ├── girona500_basic.scn
|   |        └── vehicles/
|   |             └── girona500_eca5emicro_gripper.scn  
│   └── g500/
|       ├── data/
|       |    └── ...
|       └── scenarios/
|            ├── girona500_basic.scn
|            └── vehicles/
|                 ├── girona500.scn
|                 └── girona500_eca5emicro_gripper.scn  
├── scripts/                         # Python code for each environment
│   ├── acrobot/
│   │   ├── AcrobotEnv.py
│   │   ├── acro_training.py
│   │   ├── __init__.py
│   │   ├── evaluate_acro.py
│   │   └── test_acrobot.py
│   ├── g500/
│   │   ├── G500Env.py
│   │   ├── G500_training_ppo.py
│   │   ├── __init__.py
│   │   ├── evaluate_g500.py
│   │   └── test_g500.py
│   ├── core/
│   │   ├── EnvStonefishRL.py
│   │   └── launch_stonefish.py
│   └── tests_sensors_actuators/
│       ├── G500TestEnv.py
│       └── test.py
├── src/                             # C++ source code
│   ├── main.cpp
│   └── StonefishRL.cpp
└── docs/                            # Documentation for each environment
    ├── README_acrobot.md
    ├── README_girona500.md
    ├── README_tests.md
    ├── README_installation.md
    └── README_manual.md
    
