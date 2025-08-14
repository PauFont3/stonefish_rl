# Reinforcement Learning Environments with Stonefish

This repository contains several simulation environments based on **Stonefish** and controlled via **Python**.


## Available Environments
- [AcrobotEnv](./docs/README_acrobot.md) – Control a two-link pendulum to reach a target height.
- [G500Env](./docs/README_girona500.md) – Girona500 robot with a gripper to approach a target object (Ball)-
- [G500TestEnv](./docs/README_tests.md) – Testing environment for sensors and actuators.


## Developer Guide:  
- [Installation](./docs/README_installation.md) – Setup Stonefish, build the C++ server, create the Python environment and run scenes.
- [Manual](./docs/README_manual.md) – Commands (CMD/RESET/EXIT), adding sensors/actuators, creating robots, create an env structure.


## Technologies
- **Simulator**: [Stonefish](https://github.com/patrykcieslak/stonefish)
- **Reinforcement Learning**: Gymnasium + stable-baselines3
- **Communication**: ZeroMQ (pyzmq + cppzmq)  


## Stonefish documentation
This project is built on the Stonefish simulator. For full details about the simulator itself, installation, scene format, robots, sensors/actuators, rendering, and physics options, see the [official repository](https://github.com/patrykcieslak/stonefish).

> The READMEs in this repository focus on the Python and C++ integration and the RL environments.
> For simulator-specific topics (scene syntax, available components, configuration, ...), refer to the Stonefish docs.


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
    
