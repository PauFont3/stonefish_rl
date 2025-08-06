# G500Env - Entorn de Reinforcement Learning amb el Robot Girona500

Aquest entorn implementa un escenari d’aprenentatge per reforç (Reinforcement Learning) per controlar el robot `Girona500`i el braç manipualdor `ECA5eMicro` dins d'una piscina virtual utilitzant la biblioteca de **Gymnasium**.
La piscina virtual té les dimensions de la piscina real del CIRS.

---

## Requisits previs
Abans de començar, assegura’t de tenir instal·lats els següents components:
- **Stonefish Simulator (v1.5.0 o superior)**: Has de tenir Stonefish instal·lat i funcional.
- **Python 3.10** i `pip`
- **Entorn virtual** amb les dependències del projecte (requirements.txt)
- **CMake** i un compilador C++17
- **ZeroMQ** i bindings per a C++ i Python (`pyzmq`, `cppzmq`)
- **Stable-Baselines3** (entrenament)

---

## Executar l'entorn
Els scripts de Python executen automàticament el simulador de Stonefish amb l’escena adequada. Només cal executar el script que vulguem amb l’entorn Python activat:


**A. Entrenar un Nou Agent**
Entrena un agent PPO per controlar el Girona500 utilitzant Stable-Baselines3. El millor model es guardarà a `logs/best_model.zip`.
```bash 
python scripts/g500/G500_training_ppo.py
``` 


**B. Avaluar un Agent Entrenat**
Carrega el model logs/best_model.zip i mostra el seu rendiment. Si el gripper s’aconsegueix acostar a **<= 0.5m** de la bola, l’episodi es considerarà assolit i `terminated = True`.
```bash 
python scripts/g500/evaluate_g500.py
```


**C. Fer Proves Manuals (DEBUG)**
Per provar l’entron sense model, envia accions aleatòries al robot `girona500`.
```bash 
python scripts/g500/test_G500.py
```

---

## Funcionament intern de l'entorn
1. Python (G500Env.py) crea una connexió utilitzant ZeroMQ cap al servidor C++ (StonefishRLTest).
2. En fer env.step(action), Python envia comandes amb les velocitats concretes per a cada thruster i/o servo.
3. C++ aplica les comandes, avança la simulació i retorna un JSON amb les observacions.
4. Python interpreta aquest JSON, calcula la recompensa (reward) i detecta si s'ha acabat l'episodi (terminated o truncated).

## Observacions i accions
> El número entre parèntesis indica la longitud del vector corresponent.

Una observació (obs) és un vector de 38 valors reals:
- Posició de la bola (3)
- Posició i rotació del Girona500 (6)
- Velocitat lineal i angular del Girona500 (6)
- Posició i rotació del gripper (6)
- Angles de les joints (6)
- Última acció aplicada (11)

Les accions son un vector de 11 valors reals:
- 5 Thrusters (TORQUE): rang [-10.0, 10.0]
- 6 Servos (VELOCITY): rang [-1.0, 1.0]

## Política per acabar un episodi
- terminated = True: si el gripper està **<=0.5 metres** de la bola
- truncated = True: si passen 30 segons (definits per search_time) i no ho ha aconseguit

## Reward
```bash
La funció de recompensa és:
if dist > 0.5:
    reward = -dist
else:
    reward = 0
```
Per tant, com més a prop estigui el gripper de la bola, més gran serà el reward.
