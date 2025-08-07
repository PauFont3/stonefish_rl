# AcrobotEnv – Entorn de Reinforcement Learning amb Stonefish

Aquest entorn implementa un escenari d’aprenentatge per reforç (Reinforcement Learning) per controlar un robot `Acrobot` utilitzant la biblioteca **Gymnasium**. 

La simulació física es fa amb **Stonefish** (C++) i la comunicació entre Python i el simulador es gestiona a través de **ZeroMQ**.
L’objectiu de l’agent és aprendre a balancejar l’Acrobot perquè l’extrem del seu segon braç superi una altura determinada, partint des d’una posició inicial en repòs. 

---

### 1. Executar l’Entorn 

Els scripts de Python ja executen automàticament el simulador de Stonefish amb l’escena corresponent, per tant **només cal tenir l’entorn Python activat** i executar el script que es vulgui.

**Activar l'entorn virtual**  
Desde l’arrel del projecte
```bash
source env_rl/bin/activate
```

**A. Entrenar un Nou Agent**  
Per entrenar un agent des de zero utilitzant Stable-Baselines3 (PPO): 
```bash 
python scripts/acrobot/acro_training.py
``` 

**B. Avaluar un Agent Entrenat**  
Per veure com es comporta un agent ja entrenat (assegurat que el fitxer del model existeix): 
```bash 
python scripts/acrobot/evaluate_acro.py
``` 

**C. Fer Proves Manuals (DEBUG)**  
Per provar l’entron sense model, enviant accions aleatòries: 
```bash 
python scripts/acrobot/test_acrobot.py
```

#### Notes Importants
- Quan es crida a la funció de `reset()` pot ser que l’Acrobot segueixi balancejant-se durant un temps. Això és normal i és deu la una velocitat residual del robot: el sistema intenta que el robot es quedi quiet aplicant forces en sentit contrari al moviment que porta fins aconseguir una posicó equilibrada.

---

## Com Funciona 
1. El **servidor C++ (`StonefishRLTest`)** carrega l’escena (`.xml`) i espera una connexió entrant pel port 5555.
2. El **client Python (`EnvStonefishRL.py`)** crea la connexió i la connecta al servidor a través de ZeroMQ.
3. Al cridar a `env.step(...)`, Python envia una comanda al simulador C++, p.ex:`CMD:TORQUE...`.
4. El simulador aplica el torque, avança la simulació uns quants passos (`dt`), i obté les noves lectures dels sensors.
5. Aquestes lectures es serialitzen en format JSON i s'envien de tornada a Python.
6. Python processa aquest JSON, actualitza el seu estat intern, calcula la recompensa i comprova si s'han satisfet les condicions de fi de l’episodi (`terminated` i `truncated`).
7. El mètode `reset(...)` envia una comanda especial `"RESET"` que activa la funció `robot->Reset()` en C++ per restaurar l’estat inicial del robot.
