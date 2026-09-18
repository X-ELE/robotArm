# MEMORIA TÉCNICA Y ESTADO DEL PROYECTO: BRAZO ROBÓTICO ARTICULADO
**Placa Controladora:** MKS Gen v1.4 (ATmega2560 + RAMPS 1.4 integrado)  
**Actuadores Principales:** 3x Motores Paso a Paso NEMA 17 + Drivers A4988  
**Actuador Terminal (Garra):** 1x Motor Paso a Paso 28BYJ-48 (5V) + Driver ULN2003  
**Refrigeración:** Ventilador 12V en D9  
**Fecha de Actualización:** Septiembre 2026  

---

## 1. ESTADO DE FUNCIONAMIENTO GENERAL
* **Ejes Principales (Base, Hombro, Codo):** 100% funcionales y calibrados.
  - Zócalo Z: Base giratoria (G-Code: `G1 X...`).
  - Zócalo Y: Hombro / Brazo inferior (G-Code: `G1 Y...`).
  - Zócalo X: Codo / Brazo superior (G-Code: `G1 Z...`).
  - Micropasos: 1/16 configurados con los 3 jumpers debajo de cada driver.
  - Enclave/Desactivación de torque: Operativos mediante `M17` (activar) y `M18` (desactivar).
* **Ventilador de Drivers:** Operativo en bornera D9 mediante `M106` (encender) y `M107` (apagar).
* **Garra / Pinza Efectora (Gripper):**
  - Conectada a la placa controladora mediante el puerto de expansión **AUX-2**.
  - Controlada mediante `M3 T<pasos>` (cerrar) y `M5 T<pasos>` (abrir).

---

## 2. CONEXIONADO DE LA GARRA (ULN2003 + 28BYJ-48)

### ⚠️ Aclaración de Hardware y Firmware
En el firmware Arduino (`pinout.h`), los pines asignados (`40, 59, 63, 64`) corresponden físicamente al conector **AUX-2** de la MKS Gen v1.4.
Se reemplazó la librería antigua `Stepper.h` por un controlador dedicado de **8 Medios Pasos (Half-Step)** en [`gripper.cpp`](file:///home/lms/Proyectos/robotArm/Arduino/robotArm/gripper.cpp):
- Elimina vibraciones y arrancadas bruscas con un retardo seguro (1.8 ms por medio paso).
- Mapea exactamente los pines físicos directos a la secuencia lógica `IN1 ➔ IN2 ➔ IN3 ➔ IN4`.
- Apaga automáticamente todas las bobinas al terminar cada movimiento para no calentar el motor ni consumir corriente de la línea de 5V.

### Tabla de Conexionado Definitiva

| MKS Gen v1.4 (Conector AUX-2) | Pin Físico | Señal Firmware | Conexión en Placa ULN2003 |
| :--- | :---: | :---: | :--- |
| **+5V DC** | Pin 1 (Fila inferior, Col 1) | VCC | **Pin `+` (VCC)** *(Jumper de 2 pines colocado)* |
| **GND** | Pin 2 (Fila superior, Col 1) | GND | **Pin `-` (GND)** |
| **D40** | Pin 6 (Fila superior, Col 3) | `GRIPPER_IN1` | **Pin `IN1`** |
| **D63 (A9)** | Pin 4 (Fila superior, Col 2) | `GRIPPER_IN2` | **Pin `IN2`** |
| **D59 (A5)** | Pin 3 (Fila inferior, Col 2) | `GRIPPER_IN3` | **Pin `IN3`** |
| **D64 (A10)** | Pin 5 (Fila inferior, Col 3) | `GRIPPER_IN4` | **Pin `IN4`** |

*Nota del motor:* La ficha hembra blanca de 5 cables del motor 28BYJ-48 se enchufa directo al conector blanco del ULN2003.

---

## 3. COMANDOS G-CODE IMPLEMENTADOS

| Comando | Parámetros | Descripción / Acción |
| :--- | :--- | :--- |
| `M17` | - | Enclava y activa corriente en todos los drivers NEMA 17 |
| `M18` | - | Desactiva drivers (motores suaves y libres para mover a mano) |
| `M106`| - | Enciende el ventilador de disipadores (D9) |
| `M107`| - | Apaga el ventilador de disipadores (D9) |
| `M3`  | `T0` | **Diagnóstico de LEDs**: enciende secuencialmente LED 1 ➔ 2 ➔ 3 ➔ 4 durante 500 ms cada uno |
| `M3`  | `T<pasos>` | **Cierra la garra** con movimiento Half-Step suave (ej. `M3 T600`) |
| `M5`  | `T<pasos>` | **Abre la garra** con movimiento Half-Step suave (ej. `M5 T600`) |
| `G1`  | `X.. Y.. Z..` | Movimiento coordinado de cinemática inversa (X=Base, Y=Hombro, Z=Codo) |
| `G4`  | `T<seg>` | Pausa / Dwell en segundos |

---

## 4. ARCHIVOS Y HERRAMIENTAS DE PRUEBA EN EL PROYECTO

1. **`test_hardware.py`**:
   - Panel de diagnóstico rápido por consola serie (`/dev/ttyUSB0` a 9600 baud).
   - Opciones `[1]` a `[7]` para probar ventilador, enclave de motores, movimiento eje por eje y pinza con `M3 T600` / `M5 T600`.
2. **`robot_gui.py`**:
   - Interfaz gráfica moderna en PyQt6 con control de jogging multi-modo, telemetría de cinemática inversa/directa y terminal G-code.
   - **Tres Modos de Movimiento (Pestañas con Botones 30x30 px)**:
     - **🌐 Cilíndrico / Base**: Gira la base en un arco circular sobre el plano horizontal sin alterar el radio esférico ($R = \text{cte}$). Permite giro libre completo de $-90^\circ$ a $+90^\circ$ ($180^\circ$ de barrido) sin bloqueos por límite radial.
     - **🦾 Articular (J1, J2, J3)**: Control angular directo e independiente por motor en grados mediante cinemática directa (FK), eliminando singularidades cartesianas.
     - **📐 Cartesiano (X, Y, Z)**: Movimiento lineal puro en mm con cálculo en tiempo real del recorrido lineal máximo disponible en X ($X_{máx} = \sqrt{236^2 - Y^2 - Z^2}$).
   - **Cero de Trabajo (WCS / Relativo)**: Permite fijar un origen de usuario (Tara) con tecla `Z` o botón para visualizar y medir desplazamientos relativos `(ΔX, ΔY, ΔZ)`.
   - **Asistente de Calibración HOME**: Botón `📍 Calibrar Físico (M18/M17)` para liberar motores a mano, colocar la "L" recta de referencia y enclavar a `(0, 120, 120)`.
   - **Monitoreo de Extensión Esférica**: Control del radio de trabajo físico (máx. 236 mm) con porcentaje, margen de seguridad restante y límite dinámico de carrera lineal.
   - **Posicionamiento Directo**: Entrada numérica para enviar el brazo directamente a `(X, Y, Z)` deseado.
3. **`robot_control.py`**:
   - Módulo Python / CLI para interactuar por consola y automatizaciones.
4. **`prueba_motores.txt`**:
   - Guía rápida de laboratorio con verificación de bobinas, voltajes $V_{ref}$ y conexión completa de motores y gripper.
5. **`docs/GUIA_LABORATORIO_ALUMNOS_PUESTA_EN_MARCHA.md`**:
   - Guía académica completa con checklists, fundamentos teóricos de chopper, normas de seguridad y resolución de fallas.
