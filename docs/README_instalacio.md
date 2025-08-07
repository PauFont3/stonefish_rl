# Instal·lació de l'entorn

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
python3 -m venv env_rl
source env_rl/bin/activate 

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
make -j$(nproc) # Accelera la compilació, utilitzarà els nuclis que tingui disponibles la CPU
```
> `make-j$(nproc)` Accelera la compilaciódel programa, utilitzarà els nuclis que tingui disponibles la CPU. En cas que no es vulgui utilitzar la compilació paral·lela, sempre es pot utilitzar `make` el qual només utilitzarà 1 nucli.

