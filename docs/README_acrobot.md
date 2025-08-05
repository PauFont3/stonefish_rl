# AcrobotEnv – Entorn de Reinforcement Learning amb Stonefish

Aquest entorn implementa un escenari d’aprenentatge per reforç (Reinforcement Learning) per controlar un robot `Acrobot` utilitzant la biblioteca **Gymnasium**. 

La simulació física es fa amb **Stonefish** (C++) i la comunicació entre Python i el simulador es gestiona a través de **ZeroMQ**.
L’objectiu de l’agent és aprendre a balancejar l’Acrobot perquè l’extrem del seu segon braç superi una altura determinada, partint des d’una posició inicial en repòs. 

---

## Requisits Previs

Abans de començar, assegura’t de tenir instal·lats els següents components:
- **Stonefish Simulator (v1.5.0 o superior)** 
- **CMake**: Versió 3.10 o superior.
- **Compilador de C++**: Compilador compatible amb C++17 (p.ex: `g++`).
- **ZeroMQ**: La biblioteca de ZeroMQ. (Tant en C++ com en Python: `cppzmq`, `pyzmq`)  
- **Python 3.10** amb `gymnasium`, `stable-baselines3`, etc.
- Fitxer `requirements.txt` amb les dependències de Python

---

## Instal·lació i Execució:

### 1. Clonar i Preparar l’Entorn
```bash 
# 1. Clonar el repositori
git clone “https://github.com/PauFont3/stonefish_rl.git”

# 2. Navegar al directori del projecte 
cd stonefish_rl

# 3. Crear i activar un entorn virtual per Python 
python3 -m venv env 
source env/bin/activate 

# 4. Instal·lar les dependències de Python 
pip install -r requirements.txt 
```

### 2. Compilar el Simulador C++ 
Utilitzar CMake per compilar l’executable del simulador. L’executable es dirà `StonefishRLTest` (segons el teu `CMakeLists.txt`). 
```bash 
# Desde l’arrel del projecte
mkdir -p build 
cd build 
cmake .. 
make 
```

### 3. Executar l’Entorn 

Els scripts de Python ja executen automàticament el simulador de Stonefish amb l’escena corresponent, per tant **només cal tenir l’entorn Python activat** i executar el script que es vulgui.

**Activar l'entorn virtual**
#### Desde l’arrel del projecte
```bash
source env_rl/bin/activate
```

** A. Entrenar un Nou Agent **
Per entrenar un agent des de zero utilitzant Stable-Baselines3 (PPO): 
```bash 
python scripts/acrobot/acro_training.py
``` 

** B. Avaluar un Agent Entrenat **
Per veure com es comporta un agent ja entrenat (assegurat que l’arxiu del model existeix): 
```bash 
python scripts/acrobot/evaluate_acro.py
``` 

** C. Fer Proves Manuals (DEBUG)**
Per provar l’entron sense model, enviant accions aleatòries: 
```bash 
python scripts/acrobot/test_acrobot.py
```

#### Notes Importants
- Durant l’execució, s’obrirà automàticament una finestra de simulació Stonefish amb l’escena acrobot_scene.xml carregada.
- Si després d’un reset, l’Acrobot segueix balancejant-se, pot ser per la velocitat residual del robot.





## Com Funciona 
1. El **servidor C++ (`StonefishRLTest`)** carrega l’escena i espera una connexió de Python al port 5555.
2. El **client Python (`AcrobotEnv.py`)** es connecta al servidor.
3. En cridar a `env.step(action)`, Python envia una comanda `CMD:TORQUE...` a C++.
4. C++ rep la comanda, aplica el torque, avança la simulació un nombre de passos fixos (`dt`), i obté el nou estat dels sensors.
5. C++ serialitza el nou estat a format JSON i ho envia com a resposta a Python.
6. Python rep el JSON, actualitza el seu estat intern i calcula la recompensa i les condicions de fi de l’episodi (`terminated` y `truncated`), retornant la tupla estàndard de Gymnasium.
7. El procés de `reset` funciona de forma similar, enviant una comanda `"RESET"` que desencadena una funció `robot->Reset()` en C++ para restaurar l’estat inicial del robot.
