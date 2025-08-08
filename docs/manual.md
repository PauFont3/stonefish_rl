
## Afegir un nou sensor (Scalar, no de Visió) a StonefishRL
1. Incloure la llibreria del sensor (si no existeix)  
    - A `StonefishRL.cpp` o `.h`, cal afegir el `#include ...` corresponent per cada tipus de sensor.
    - Per exemple: ` #include <Stonefish/sensors/scalar/IMU.h> `

2. Definir el sensor al XML de l'escena  
    - Afegir el nou sensor a l'escena (`.scn`).

3. Afegir-lo a `InfoObject` (StonefishRL.h)  
    - Afegir al struct `struct InfoObject`les noves dades del sensor.

4. Comprovar que Stonefish el detecta (StonefishRL.cpp)
    - A través del mètode `ProvaMostrarTot()` permet veure per la terminal si el sensor apareix i quins canals d'informació té.

5. Afegir-lo al vector d'observacions  
    - A `GetStateScene()`, ja hi ha varis sensors afegits, caldrà seguir el mateix patró per afegir l'informació que volem recollir i enviar cap al Python.   
  **Important**: Fer el `push_back` perque s'afegeixin les observacions a la tupla (InfoObject).

6. Enviar les dades a Python  
    - Al mètode `InfoObjectToJson(...)` cal afegir les dades utilitxant el mètode 'SafeFloat(...)` per deixar com a `nan` els camps que no interessen.  
  Al mètode `FillWithNanInfoObject(...)`, cal inicialitzar les dades que volguem recollir del sensor que afeguim amb `nan`. 

7. Verificar que Pyhton ha rebut els valors  
    - Descomentar la linea de codi `self.print_full_state()` de la funció `step(self, message, steps)` a `EnvStonefishRL`, permeten així mostrar per pantalla els
      valors que s'han obtingut del sensor afegit.  
