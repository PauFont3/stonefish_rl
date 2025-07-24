# Entorn de Reinforcement Learning amb Stonefish

Aquest projecte implementa un entorn d’aprenentatge per reforç (Reinforcement Learning) per entrenar un robot `Acrobot` utilitzant la biblioteca **Gymnasium**. 
La simulació física es fa amb **Stonefish** (C++) i la comunicació  entre Python i el simulador es fa a través de **ZeroMQ**. 
L’objectiu de l’agent és aprendre a balancejar l’Acrobot perquè l’extrem del 2n braç (segona articulació) superi una altura determinada, partint d’una posició inicial en repòs vertical. 

## Requisits Previs

Abans de començar, assegura’t de tenir instal·lats els següents components:
- **Stonefish Simulator (v1.5.0 o superior)**: És necessari tenir Stonefish instal·lat al teu sistema. Asegura’t que les variables d’entorn (`STONEFISH_PATH`, etc.) estiguin configurades correctament.
- **CMake**: Versió 3.10 o superior.
- **Compilador de C++**: Compilador compatible amb C++17 (p.ex: `g++`).
- **ZeroMQ**: La biblioteca de ZeroMQ.
- **Python 3.10** i `pip`. 

## Instal·lació i Execució:
Per compilar i executar el projecte:

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
S’han d’obrir **2 terminals**. 

**Terminal 1: Iniciar el Simulador C++** 
En aquesta terminal, executa el simulador. Ha d’estar obert mentre utilitzes els scripts de Python. 
#### Desde l’arrel del projecte
```bash  
./build/StonefishRLTest /home/ubuntu/stonefish_rl/Resources/acrobot/acrobot_scene.xml
``` 
Si tot va bé, veuràs la finestra de la simulació i els logs de Stonefish. 

**Terminal 2: Executar els Scripts de Python** 
En aquesta 2a terminal, assegurat de tenir l’entorn virtual de Python activat (`source env/bin/activate`). 

#### A. Entrenar un Nou Agent
Per entrenar un agent des de zero utilitzant Stable-Baselines3 (PPO): 
```bash 
python python/acro_learning.py 
``` 

El progrés de l’entrenament es mostrarà a la terminal. En finalitzar, es guardarà el model entrenat (p.ex: `ppo_acrobot_stonefish.zip`). 

#### B. Avaluar un Agent Entrenat 
Per veure com es comporta un agent ja entrenat (assegurat que l’arxiu del model existeix): 
```bash 
python python/evaluate_acro.py 
``` 
Aquest script carregarà el model guardat i l’executarà a l’entorn, mostrant el seu rendiment. 

#### C. Fer Proves Manuals
Per provar l’entron sense model amb l’entorn enviant accions aleatòries (útil per depuració): 
```bash 
python python/test_acrobot.py 
```

#### Notes Importants
- Si després d’un reset, l’Acrobot segueix balancejant-se, pot ser per la velocitat residual del robot.





## Com Funciona 
1. El **servidor C++ (`StonefishRLTest`)** carrega l’escena i espera una connexió de Python al port 5555.
2. El **client Python (`AcrobotEnv.py`)** es connecta al servidor.
3. En cridar a `env.step(action)`, Python envia una comanda `CMD:TORQUE...` a C++.
4. C++ rep la comanda, aplica el torque, avança la simulació un nombre de passos fixos (`dt`), i obté el nou estat dels sensors.
5. C++ serialitza el nou estat a format JSON i ho envia com a resposta a Python.
6. Python rep el JSON, actualitza el seu estat intern i calcula la recompensa i les condicions de fi de l’episodi (`terminated` y `truncated`), retornant la tupla estàndard de Gymnasium.
7. El procés de `reset` funciona de forma similar, enviant una comanda `"RESET"` que desencadena una funció `robot->Reset()` en C++ para restaurar l’estat inicial del robot.
