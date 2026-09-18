# GUÍA DE PRÁCTICA DE LABORATORIO
## PUESTA EN MARCHA, CALIBRACIÓN Y DIAGNÓSTICO DE HARDWARE
### Brazo Robótico Articulado - Controlador MKS Gen v1.4

---

**Asignatura:** Robótica Industrial / Mecatrónica / Sistemas de Control  
**Puesto de Trabajo / Banco N°:** _________  
**Integrantes del Grupo:**  
1. __________________________________________________  
2. __________________________________________________  
3. __________________________________________________  
**Fecha de Realización:** ____ / ____ / 202__   **Calificación:** _________  

---

## 1. OBJETIVOS DE LA PRÁCTICA

Al finalizar esta práctica, el estudiante será capaz de:
1. Identificar y verificar las conexiones de potencia, control y actuadores en una placa controladora **MKS Gen v1.4**.
2. Identificar pares de bobinas en motores paso a paso bipolares NEMA 17 mediante multímetro digital.
3. Comprender el funcionamiento de los controladores conmutados (*chopper*) y **calibrar con precisión el voltaje de referencia ($V_{ref}$)** en drivers A4988 / DRV8825.
4. Ejecutar pruebas dinámicas aisladas por software y diagnosticar fallas típicas (pérdida de pasos, sobrecalentamiento, inversión de giro).

---

## 2. NORMAS CRÍTICAS DE SEGURIDAD ELÉCTRICA

> [!CAUTION]
> **REGLA DE ORO DE LOS CONTROLADORES PASO A PASO:**
> **JAMÁS** conecte ni desconecte el cable de un motor paso a paso con la fuente de 12V encendida. La desconexión en caliente genera un arco de fuerza contraelectromotriz (f.c.e.m.) que **quema instantáneamente el driver A4988**. Antes de tocar cualquier cable, apague la fuente.

> [!WARNING]
> **TRAMPA DE MEDICIÓN DE CORRIENTE:**
> Los drivers A4988 son fuentes conmutadas reductoras (*buck chopper*). **NO intente medir la corriente de los motores conectando un amperímetro en serie con la fuente de 12V buscando 1.0 A**. 
> Si la fuente de 12V marca 1.0 A para un solo motor, por conservación de potencia la bobina estará recibiendo más de **4 Amperios**, destruyendo el driver y quemando el motor. **La corriente se calibra exclusivamente mediante la medición de $V_{ref}$ en Voltios DC**.

---

## 3. INSTRUMENTAL Y MATERIALES REQUERIDOS

* 1x Brazo robótico con placa MKS Gen v1.4 y 4x drivers A4988 (o DRV8825).
* 1x Fuente de alimentación regulada 12V DC (mínimo 5A).
* 1x Multímetro digital con puntas de prueba fina.
* 1x Destornillador plano de ajuste fino (preferentemente cerámico o con mango aislado).
* 1x Cable USB tipo B a PC con Linux / Windows.
* Lupa o cámara de smartphone (para inspección de resistencias SMD).

---

## 4. PROCEDIMIENTO EXPERIMENTAL

### FASE I: Inspección con Placa Desenergizada (Checklist Inicial)

*Realice las siguientes verificaciones con la fuente de 12V y el cable USB completamente desconectados. Marque con una cruz cada punto verificado:*

- [ ] **Alimentación:** Bornera verde `12V/24V IN`. Polaridad verificada: borne positivo (+) conectado a +12V y borne negativo (-) conectado a masa (GND). Cables firmemente ajustados sin filamentos sueltos.
- [ ] **Configuración de Micropasos:** Se retiraron temporalmente los drivers y se verificó que los **3 jumpers (MS1, MS2, MS3)** debajo de cada zócalo (`X`, `Y`, `Z`, `E0`) estén colocados. Esto configura **1/16 de micropaso** (3200 pasos/vuelta).
- [ ] **Orientación de los Drivers:** Se verificó que los pines `DIR` y `GND` de la plaquita del driver coincidan exactamente con la serigrafía `DIR` y `GND` de la placa MKS Gen v1.4. *(¡Un driver invertido se destruye al encender!)*.
- [ ] **Mapeo de Motores:** Se conectaron los cables de 4 pines en sus zócalos correctos:
   - Zócalo **Z** $\rightarrow$ Motor de la Base giratoria
   - Zócalo **Y** $\rightarrow$ Motor del Hombro (brazo inferior)
   - Zócalo **X** $\rightarrow$ Motor del Codo (antebrazo)
   - Zócalo **E0** $\rightarrow$ *(Sin motor de muñeca instalado / Libre)*
   - Conector **AUX-2** $\rightarrow$ Alimentación 5V/GND y pines D40, D63, D59, D64 hacia driver ULN2003 de la pinza (28BYJ-48). *(Nota: En pinout.h figura '//RAMPS AUX-1' por error histórico, pero corresponden físicamente a AUX-2. Para que la librería Stepper.h active las fases en secuencia correcta, conectar IN1 a D40, IN2 a D63, IN3 a D59 e IN4 a D64)*.
   - Bornera **D9** $\rightarrow$ Ventilador de 12V apuntando a los disipadores

---

### FASE II: Identificación de Bobinas de los Motores NEMA 17

*Los motores paso a paso bipolares poseen dos bobinas independientes (Fase A y Fase B). Para que el motor gire suavemente, cada par de cables debe corresponder a una misma bobina.*

1. Desconecte el motor de la placa.
2. Coloque el multímetro en **modo continuidad (bip) o medición de resistencia ($200\,\Omega$)**.
3. Mida entre los pines del conector de 4 terminales:
   - Los dos terminales que den continuidad (típicamente entre $1.5\,\Omega$ y $4.0\,\Omega$) pertenecen a la **Bobina 1**.
   - Los otros dos terminales que den continuidad pertenecen a la **Bobina 2**.
   - Entre la Bobina 1 y la Bobina 2 la resistencia debe ser **infinita (circuito abierto)**.

**Registro de Mediciones de Bobinas:**
* Motor Base (Z): Resistencia Fase A: ________ $\Omega$ | Resistencia Fase B: ________ $\Omega$
* Motor Hombro (Y): Resistencia Fase A: ________ $\Omega$ | Resistencia Fase B: ________ $\Omega$
* Motor Codo (X): Resistencia Fase A: ________ $\Omega$ | Resistencia Fase B: ________ $\Omega$

---

### FASE III: Calibración del Voltaje de Referencia ($V_{ref}$)

#### Fundamento Teórico:
Para drivers A4988 con resistencia sensora de corriente $R_{\text{sense}}$, la corriente de pico por fase viene dada por:
$$I_{\text{max}} = \frac{V_{ref}}{8 \times R_{\text{sense}}} \quad \Longrightarrow \quad V_{ref} = I_{\text{deseada}} \times 8 \times R_{\text{sense}}$$

1. **Inspección Visual:** Observe las dos pequeñas resistencias rectangulares negras al lado del chip del driver. Lea el código impreso:
   - Si dice **`R100`**, la resistencia es de **$0.10\,\Omega$** $\rightarrow$ **$V_{ref} = 0.8 \times I_{\text{deseada}}$**.
   - Si dice **`R050`**, la resistencia es de **$0.05\,\Omega$** $\rightarrow$ **$V_{ref} = 0.4 \times I_{\text{deseada}}$**.
2. **Procedimiento de Medición:**
   - Conecte el cable USB a la PC y encienda la fuente de 12V.
   - Ponga el multímetro en **Voltaje Continuo (Escala DC 2V o 20V)**.
   - Coloque la **punta negra (COM)** firmemente en el contacto metálico exterior del conector USB o en el borne GND de la fuente.
   - Apoye suavemente la **punta roja** en la cabeza metálica del tornillo del potenciómetro de ajuste del driver.
   - Con el destornillador, gire muy lentamente el tornillo hasta alcanzar el valor objetivo. *(Giro horario suele aumentar; antihorario disminuye).*

#### Tabla de Registro de Calibración de Corriente:

| Eje / Zócalo | Driver | Código $R_s$ | $I_{\text{objetivo}}$ | $V_{ref}$ Teórico | $V_{ref}$ Medido Final | Firma Verificación Docente |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Z (Base)** | A4988 | | 0.90 A | | | |
| **Y (Hombro)** | A4988 | | 1.00 A | | | |
| **X (Codo)** | A4988 | | 1.00 A | | | |
| **E0 (Muñeca)** | *(No montada)* | - | - | - | N/A | Docente: Omitir |

---

### FASE IV: Pruebas Dinámicas Individuales por Software

*Una vez verificada la calibración eléctrica, se procede a las pruebas mecánicas aisladas.*

1. Abra una terminal en la PC y navegue hasta la carpeta del proyecto:
   ```bash
   cd /home/lms/Proyectos/robotArm
   python3 test_hardware.py
   ```
2. Ejecute en orden cada una de las siguientes opciones del menú interactivo y anote sus observaciones:

#### Checklist de Pruebas Dinámicas:
- [ ] **Opción [1] Prueba del Ventilador (D9):** ¿El ventilador encendió durante 3 segundos con buen flujo de aire hacia los disipadores? *(Sí / No)*: _________
- [ ] **Opción [2] Prueba de Enclave (`M17`):** Al activar los motores, intente mover suavemente cada eje con la mano. ¿Se encuentran todos bloqueados con torque de retención firme? *(Sí / No)*: _________
- [ ] **Opción [3] Prueba de Desactivación (`M18`):** Al desactivar los motores, ¿quedan todos los ejes completamente libres y suaves para moverlos a mano? *(Sí / No)*: _________
- [ ] **Opción [4] Movimiento Base (Z):** ¿La base giró a la derecha e izquierda sin tirones ni pérdidas de paso? *(Correcto / Invierte giro / Traba)*: _________
- [ ] **Opción [5] Movimiento Hombro (Y):** ¿El brazo inferior se desplazó adelante y atrás suavemente? *(Correcto / Invierte giro / Traba)*: _________
- [ ] **Opción [6] Movimiento Codo (X):** ¿El brazo superior subió y bajó correctamente? *(Correcto / Invierte giro / Traba)*: _________
- [ ] **Opción [7] Movimiento Pinza (Gripper):** ¿La pinza abrió y cerró completamente? *(Sí / No)*: _________

3. **Verificación Térmica:** Deje los motores enclavados (`M17`) durante 5 minutos. Toque el cuerpo metálico de los motores con el dorso del dedo:
   - Motor Base: [ ] Frío [ ] Tibio (~40°C) [ ] Caliente quemante (>70°C)
   - Motor Hombro: [ ] Frío [ ] Tibio (~40°C) [ ] Caliente quemante (>70°C)
   - Motor Codo: [ ] Frío [ ] Tibio (~40°C) [ ] Caliente quemante (>70°C)

---

## 5. GUÍA DE RESOLUCIÓN DE FALLAS (TROUBLESHOOTING)

Si durante las pruebas ocurre algún problema, consulte la siguiente tabla antes de llamar al docente:

| Falla Observada | Causa Raíz Probable | Acción Correctiva Obligatoria |
| :--- | :--- | :--- |
| **El motor vibra intensamente y hace ruido de "zumbido" pero no gira.** | Bobinas cruzadas en el conector (el cableado no es AABB). | **Apague la fuente de 12V**. Revise con el tester la continuidad y reconecte los terminales para asegurar que la bobina 1 esté en pines 1-2 y la bobina 2 en pines 3-4. |
| **El motor gira en sentido contrario al indicado.** | Polaridad de bobina invertida. | **Apague la fuente de 12V**. Retire el conector de 4 pines del motor, gírelo 180° y vuelva a insertarlo. |
| **Un motor queda completamente blando (sin torque) al enviar `M17`.** | Falso contacto en pines o driver descalibrado/dañado. | Verifique que el conector esté firme. Mida $V_{ref}$; si marca 0.0V, el potenciómetro está al mínimo o el driver está quemado. |
| **El motor pierde pasos o se frena a mitad de recorrido.** | Exceso de apriete en tornillos mecánicos o $V_{ref}$ insuficiente. | Mueva el brazo a mano con la máquina apagada. Si está duro, afloje 1/4 de vuelta los tornillos de los rodamientos. Si está suave, suba $V_{ref}$ un 10%. |
| **El motor o driver quema al tacto en menos de 2 minutos.** | $V_{ref}$ excesivo. | Baje inmediatamente el potenciómetro con el multímetro a $0.60\text{ V}$. Verifique que el ventilador de refrigeración esté funcionando. |
| **La pinza (ULN2003 / 28BYJ-48) enciende los LEDs pero sólo vibra y no gira.** | Fases intermedias invertidas por secuencia de `Stepper.h` de Arduino. | **Intercambie los cables de IN2 e IN3 en la placa ULN2003** (IN1 $\rightarrow$ D40, IN2 $\rightarrow$ D63, IN3 $\rightarrow$ D59, IN4 $\rightarrow$ D64). Verifique que el jumper de alimentación del ULN2003 esté colocado y pruebe con `M3 T600`. |

---

## 6. CUESTIONARIO DE EVALUACIÓN CONCEPTUAL

*Responda de forma clara y fundamentada:*

1. ¿Por qué es destructivo desconectar el conector de un motor paso a paso mientras la placa está recibiendo alimentación de 12V? Explique el fenómeno electromagnético involucrado.  
   _________________________________________________________________________________________________________  
   _________________________________________________________________________________________________________  

2. Si un motor NEMA 17 tiene una resistencia por bobina de $2\,\Omega$ y se calibra para una corriente de $1.0\text{ A}$, ¿cuánta potencia disipa la bobina? Si la fuente entrega $12\text{ V}$, ¿por qué la corriente que entrega la fuente de alimentación no es de $1.0\text{ A}$?  
   _________________________________________________________________________________________________________  
   _________________________________________________________________________________________________________  

3. ¿Qué función cumplen los jumpers de micropasos debajo del driver A4988 y qué diferencia existe entre operar a paso completo (*Full Step*) y a 1/16 de micropaso?  
   _________________________________________________________________________________________________________  
   _________________________________________________________________________________________________________  

---

**Conclusión del Grupo sobre el Estado del Robot:**  
_____________________________________________________________________________________________________________  
_____________________________________________________________________________________________________________  

**Firma del Docente a Cargo:** _________________________ **Fecha:** ____ / ____ / 202__
