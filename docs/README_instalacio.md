# Environment Installation

## Prerequisites

Before you start, make sure the **Stonefish** simulator (version **1.5.0** or higher) is correctly installed and runs without errors.  
You can find the repository and installation instructions here: [Stonefish](https://github.com/patrykcieslak/stonefish)

---

## Prepare the system (Ubuntu 22.04 recommended)
> If you use **Ubuntu 20.04**, ensure **Python 3.10** is installed, as it is not included by default.

```bash
# 1. Update system packages
sudo apt update
sudo apt upgrade

# 2. Install Python 3.10 and virtual environment tools
sudo apt install python3.10
sudo apt install python3.10-venv
sudo apt install python3-pip

# 3. Install git (if not already installed)
sudo apt install git

# 4. Install CMake (v3.10 or higher) and a C++ 17-compatible compiler (e.g., g++)
sudo apt install cmake
sudo apt install g++

# 5. Install the ZeroMQ library for C++
sudo apt install libzmq3-dev

# 6. Install cppzmq (C++ bindings)
git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
mkdir build
cd build
cmake ..
sudo make install  # <--- May fail on some systems (see Note)

# 7. (Optional) Return to the previous directory
cd ../..
```

> ### Note on `cppzmq` and installation
> `cppzmq` is a *header-only* library, only `.hpp` headers, no binaries to build.  
> On some systems, `sudo make install` may fail because it attempts to build and install bundled tests/examples.  
> If this happens, a solution that works is to run `sudo cmake --install .`, which installs only the required headers (and config files) without compiling extras, allowing you to use the library by adding `#include <zmq.hpp>`.

---

## Project Installation

### 1. Clone and set up the environment

```bash 
# 1. Clone the repository
git clone “https://github.com/PauFont3/stonefish_rl.git”

# 2. Move into the project directory
cd stonefish_rl

# 3. Create and activate a Python virtual environment
python3.10 -m venv env_rl
source env_rl/bin/activate 

# 4. Install Python dependencies
pip install -r requirements.txt 
```
> Reminder: If you close the terminal, you’ll need to reactivate the environment with: `source env_rl/bin/activate`

### 2. Build the C++ simulator
Use CMake to build the simulator executable. The executable will be named `StonefishRLTest` (as specified in the provided `CMakeLists.txt`).

```bash 
# From the project root
mkdir -p build 
cd build 
cmake .. 
make -j$(nproc)
```

> `make -j"$(nproc)"` speeds up the compilation by using all available CPU cores.  
> If you prefer not to use parallel compilation, simply run `make` (single core).
