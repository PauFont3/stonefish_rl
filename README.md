# Entorns d’Aprenentatge per Reforç amb Stonefish

Aquest repositori conté diversos entorns de simulació basats en **Stonefish** i controlats a través **Python + Reinforcement Learning**.

## Entorns disponibles

- [AcrobotEnv](./docs/README_acrobot.md) – Control d’un pèndol amb 2 articulacions per superar una altura determinada
- [G500Env](./docs/README_g500.md) – Robot Girona500 amb un braç per apropar-se a un objecte (Ball)
- [G500TestEnv](./docs/README_g500test.md) – Entorn de test per sensors i actuadors

## Tecnologies utilitzades
- **Simulador**: [Stonefish](https://github.com/patrykcieslak/stonefish)
- **Reinforcement Learning**: Gymnasium + stable-baselines3
- **Comunicació**: ZeroMQ (pyzmq + cppzmq)

## 📁 Estructura del projecte

```bash
stonefish_rl/
├── include/                         # Headers de C++
│   └── StonefishRL.h
├── logs/                            # Models PPO i arxius d’avaluació de l'entrenament
│   ├── best_model.zip
│   └── evaluations.npz
├── ppo_acrobot_stonefish_50000_5_4__7_0.zip                        # Model entrenat de l' Acrobot
├── requirements.txt                 # Dependències Python
├── Resources/                       # Escenes, models i textures per Stonefish
│   ├── acrobot/
│   │   └── acrobot_scene.xml
│   ├── learning/
│   ├── minimal/
│   ├── tests_sensors_actuators/
│   └── udg_cirs-iauv_simulation/
├── scripts/                         # Codi Python per cada entorn
│   ├── acrobot/
│   │   ├── AcrobotEnv.py
│   │   ├── acro_training.py
│   │   └── ...
│   ├── g500/
│   │   ├── G500Env.py
│   │   ├── G500_training_ppo.py
│   │   └── ...
│   ├── core/
│   │   ├── EnvStonefishRL.py
│   │   └── launch_stonefish.py
│   └── tests_sensors_actuators/
│       ├── G500TestEnv.py
│       └── test.py
├── src/                             # Codi font en C++
│   ├── main.cpp
│   └── StonefishRL.cpp
├── build/                           # Carpeta de compilació del simulador (StonefishRLTest) (NO apareix al repositori)
└── docs/                            # Documentació per cada entorn
    ├── README_acrobot.md
    ├── README_g500.md
    └── README_g500test.md
