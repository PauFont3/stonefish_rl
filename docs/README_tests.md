## G500TestEnv - Entorn de Test amb Girona500

Aquest entorn (`G500TestEnv`) està pensat per fer proves amb el robot Girona500 dins d'una piscina virtual a Stonefish, però no te cap objectiu d'aprenentatge en concret.
És útil per comprovar que els actuadors, sensors i/o robots funcionen correctament, que es poden enviar comandes i rebre observacions, i que es poden fer passos a l'entorn.

---

## Executar l'entorn:
Assegura’t de tenir l’entorn virtual activat:
```bash
source env_rl/bin/activate
```

**Executar el test:**
```bash
python scripts/tests_sensors_actuadors/test.py
```
Això fara:
1. Executa automàticament el simulador de Stonefish amb l'escena de tests (g500_basic.scn).
> L'escena `(g500_basic.scn)` té el mateix nom que l'escena de l'entorn del girona500 amb RL perque vaig reaprofitar tota l'escena del girona500 que ja estava creada, per modificar i afegir els sensors i actuadors que estava provant en una escena ja funcional.

2. Es crida l'entorn G500TestEnv.
3. Fa un `reset()` que col·loca el robot (`girona500`) i l'objecte "Ball" a unes posicions inicials aleatories.
4. Entra en un bucle on s'envien continuamentaccions aleatòries i es fa un `step()` continuament.

---

## Ús de l'entorn:

Aquest entorn està pensat per:
- Comprovar que sensors i actuadors funcionen bé després de modificar l'escena o el el codi.
- Ajustar rangs d'accions o observacions.
- Comprovar si el simulador respon correctament als nous elements afegits.
- Comprovar si la posició i moviments del robot son coherents.
