# Instal·lació de l'entorn

## Requisits Previs

Abans de començar, assegura’t de tenir instal·lats al sistema els següents components:
- **ZeroMQ**: La biblioteca de comunicació entre C++ i Python (`libzmq3-dev`, `cppzmq`, `pyzmq`).
- **Stonefish** (v1.5.0 o superior) Simulador instal·lat i funcional. 

---

## Preparar el sistema (Recomanat utilitzar Ubuntu 22.04)
> Si s'utilitza **Ubuntu 20.04**, cal assegurar-se de tenir instal·lat Python 3.10, ja que no ve per defecte al sistema.

```bash
# 1. Actualitzar els paquets del sistema
sudo apt update
sudo apt upgrade

# 2. Instal·lar Python 3.10 i eines pels entorns virtuals
sudo apt install python3.10
sudo apt install python3.10-venv
sudo apt install python3-pip

# 3. Instal·lar git (si no està ja instal·lat)
sudo apt install git

# 4. Instal·lar CMake (Versió 3.10 o superior) i un compilador compatible amb C++17 (p.ex: g++).
sudo apt install cmake
sudo apt install g++

# 5. Instal·lar la biblioteca ZeroMQ per C++
sudo apt install libzmq3-dev

# 6. Instal·lar cppzmq (bindings per C++)
git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
mkdir build
cd build
cmake ..
sudo make install  # <--- Pot fallar en alguns entrorns (veure Nota)

# 7. (Opcional) Tornar al directori anterior
cd ../..
```
> ### Nota sobre `cppzmq` i la instal·lació  
> `cppzmq` és una biblioteca *header-only*, és a dir, només té fitxers amb capçalera (.hpp) i no necessita compilar fitxers binaris.  
> En alguns sistemes, la comanda `sudo make install` pot fallar perquè intenta compilar i instal·lar tests i exemples que hi ha inclosos al repositori.  
> Si es dona aquest cas, una solució que m'ha funcionat és utilitzar la comanda `sudo cmake --install .`, que només instal·la els headers necessaris (i arxius config) sense compilar res extra, evitant errors i permetent executar la llibreria incloent `#include <zmq.hpp>`.

---

## Instal·lació del projecte

### 1. Clonar i Preparar l’Entorn

```bash 
# 1. Clonar el repositori
git clone “https://github.com/PauFont3/stonefish_rl.git”

# 2. Navegar al directori del projecte 
cd stonefish_rl

# 3. Crear i activar un entorn virtual per Python 
python3.10 -m venv env_rl
source env_rl/bin/activate 

# 4. Instal·lar les dependències de Python 
pip install -r requirements.txt 
```
> Recordatori: Si es tanca la terminal, s'haurà de reactivar l'entorn amb: `source env_rl/bin/activate`

### 2. Compilar el Simulador C++ 
Utilitzar CMake per compilar l’executable del simulador. L’executable es dirà `StonefishRLTest` (segons el `CMakeLists.txt` que he creat). 
```bash 
# Desde l’arrel del projecte
mkdir -p build 
cd build 
cmake .. 
make -j$(nproc)
```
> `make -j$(nproc)` Accelera la compilació del programa, utilitzarà els nuclis que tingui disponibles la CPU. En cas que no es vulgui utilitzar la compilació paral·lela, sempre es pot utilitzar `make` el qual només utilitza 1 nucli.

