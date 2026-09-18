# 🦾 Brazo Robótico Articulado 3DOF + Pinza Efectora

[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![PyQt6](https://img.shields.io/badge/GUI-PyQt6-41CD52?logo=qt&logoColor=white)](https://riverbankcomputing.com/software/pyqt/)
[![Hardware](https://img.shields.io/badge/Board-MKS%20Gen%20v1.4-orange)](https://reprap.org/wiki/MKS_GEN)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Ready-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Plataforma integral de control, cinemática y teleoperación para un brazo robótico articulado de 3 grados de libertad (Base, Hombro, Codo) con pinza efectora (*gripper*), gobernado por una controladora industrial **MKS Gen v1.4** (ATmega2560 + drivers A4988) y software de control en **Python 3 / PyQt6**.

Basado en la geometría y diseño mecánico original de **Florin Tobler** (*Thingiverse #1718984*) y adaptado con cinemática directa/inversa bidireccional, múltiples modos de jogging, controlador *half-step* de bajo consumo para la pinza, y puente listo para **ROS 2**.

---

## 📑 Tabla de Contenidos
1. [Estructura del Proyecto](#-estructura-del-proyecto)
2. [Arquitectura de Hardware y Conexiones](#-arquitectura-de-hardware-y-conexiones)
3. [Cinemática y Espacio de Trabajo](#-cinemática-y-espacio-de-trabajo)
4. [Software y Herramientas](#-software-y-herramientas)
5. [Puesta en Marcha Rápida (Quickstart)](#-puesta-en-marcha-rápida-quickstart)
6. [Comandos G-Code Soportados](#-comandos-g-code-soportados)
7. [Plan de Acción y Hoja de Ruta (Roadmap)](#-plan-de-acción-y-hoja-de-ruta-roadmap)

---

## 📂 Estructura del Proyecto

El repositorio se encuentra organizado de forma modular para separar firmware, modelos CAD, documentación, interfaces y puente robótico:

```
robotArm/
├── Arduino/                   # Firmware C++ para MKS Gen v1.4 / RAMPS 1.4
│   └── robotArm/              # Sketch principal, cinemática, interpolación y gripper
├── cad/                       # Archivos mecánicos, diseño 3D y documentación de armado
│   ├── assembly_guide/        # Guía de ensamblaje en PDF (versión correa/belt)
│   ├── original_stl/          # Modelos STL originales de piezas para impresión 3D
│   └── zips/                  # Archivos comprimidos de modelos de Thingiverse
├── docker/                    # Entorno Dockerizado para ROS 2 Humble
├── docs/                      # Documentación técnica, académica y guías
│   ├── MEMORIA_PROYECTO.md    # Estado técnico exhaustivo del proyecto y cableado
│   ├── GUIA_LABORATORIO...    # Manual para alumnos de laboratorio y puesta en marcha
│   ├── GUIA_ROBOTICA...       # Guía de robótica industrial y arquitectura ROS
│   └── prueba_motores.txt     # Guía rápida de laboratorio y ajuste de Vref
├── legacy/                    # Software histórico original en Delphi 7 (Florin Tobler, 2016)
│   ├── ControlDelphi/         # Código fuente Delphi
│   └── ControlDelphiBinary/   # Ejecutables binarios clásicos (.exe)
├── PRPs/                      # Product Requirements Prompts (Context Engineering)
├── ros2_ws/                   # Workspace ROS 2 (Paquetes bridge serial y descripción URDF)
│   └── src/
│       ├── robot_arm_bridge/      # Nodo puente serial entre ROS 2 y MKS Gen v1.4
│       └── robot_arm_description/ # Modelo URDF/Xacro, meshes STL y launch files
├── robot_gui.py               # Interfaz gráfica moderna en PyQt6 (Centro de control)
├── robot_control.py           # Módulo Python y CLI para automatizaciones
├── test_hardware.py           # Herramienta de diagnóstico rápido individual de hardware
└── README.md                  # Este documento
```

---

## ⚡ Arquitectura de Hardware y Conexiones

### Placa y Actuadores Principales
* **Controladora:** MKS Gen v1.4 (Alimentación: 12V DC).
* **Motores Principales:** 3x NEMA 17 bipolares con drivers A4988 configurados a **1/16 micropasos** (los 3 jumpers colocados bajo cada driver).
  * **Zócalo Z:** Motor de Base Giratoria (G-Code: `G1 X...`).
  * **Zócalo Y:** Motor de Hombro / Brazo Inferior (G-Code: `G1 Y...`).
  * **Zócalo X:** Motor de Codo / Brazo Superior (G-Code: `G1 Z...`).
* **Refrigeración:** Ventilador de 12V para disipadores conectado en bornera **D9** (`M106` / `M107`).

### Conexión de la Pinza (ULN2003 + 28BYJ-48)
La pinza terminal utiliza un motor unipolar **28BYJ-48 (5V)** conectado a su placa controladora **ULN2003**, la cual se conecta directamente a los pines de expansión **AUX-2** de la MKS Gen v1.4:

| MKS Gen v1.4 (Conector AUX-2) | Pin Placa | Función en Firmware | Pin en Driver ULN2003 |
| :--- | :---: | :---: | :--- |
| **+5V DC** | Pin 1 (Fila inf, Col 1) | VCC | **Pin `+` (VCC)** *(con jumper colocado)* |
| **GND** | Pin 2 (Fila sup, Col 1) | GND | **Pin `-` (GND)** |
| **D40** | Pin 6 (Fila sup, Col 3) | `GRIPPER_IN1` | **Pin `IN1`** |
| **D63 (A9)** | Pin 4 (Fila sup, Col 2) | `GRIPPER_IN2` | **Pin `IN2`** |
| **D59 (A5)** | Pin 3 (Fila inf, Col 2) | `GRIPPER_IN3` | **Pin `IN3`** |
| **D64 (A10)** | Pin 5 (Fila inf, Col 3) | `GRIPPER_IN4` | **Pin `IN4`** |

> [!NOTE]
> El firmware incluye un controlador unipolar dedicado de **8 Medios Pasos (Half-Step)** en `gripper.cpp`. Este controlador elimina tirones, reduce el calentamiento apagando las bobinas al finalizar el movimiento y proporciona un test de diagnóstico de LEDs (`M3 T0`).

---

## 📐 Cinemática y Espacio de Trabajo

* **Longitud de Eslabones:** Brazo inferior = $120\,\text{mm}$, Brazo superior = $120\,\text{mm}$.
* **Alcance Físico Máximo:** $240.0\,\text{mm}$ (100% de extensión recta).
* **Posición HOME de Referencia:** $(X = 0, Y = 120, Z = 120)\,\text{mm}$ formando una "L" recta ($0^\circ$ Base, $0^\circ$ Hombro vertical, $-90^\circ$ Codo horizontal).
* **Geometría Esférica:** El límite de alcance es una esfera respecto al pivote del hombro:
  $$R = \sqrt{X^2 + Y^2 + Z^2} \le 236.0\,\text{mm}\quad \text{(98\% del radio físico)}$$

---

## 🖥️ Software y Herramientas

### 1. `robot_gui.py` (Panel de Control Gráfico en PyQt6)
Punto de entrada visual para el control del robot. Incluye:
* **Botones Compactos ($30\times 30\,\text{px}$):** Disposición ordenada y ergonómica.
* **3 Modos de Movimiento (Jogging):**
  1. **🌐 Modo Cilíndrico / Base (Recomendado):** Rota la base en un arco circular sobre el plano horizontal sin alterar el radio esférico ($R = \text{cte}$). Permite un giro libre de **$-90^\circ$ a $+90^\circ$ ($180^\circ$ de barrido)** sin topar con el límite esférico.
  2. **🦾 Modo Articular:** Control directo en grados motor por motor (**J1 Base**, **J2 Hombro**, **J3 Codo**) mediante cinemática directa (FK) exacta.
  3. **📐 Modo Cartesiano:** Movimiento lineal puro en $X, Y, Z$ con indicador dinámico del recorrido máximo disponible en $X$ según $Y$ y $Z$.
* **Cero de Trabajo (WCS / Tara):** Tecla rápida `Z` o botón para fijar un origen relativo $\Delta X, \Delta Y, \Delta Z$ para medir distancias netas.
* **Asistente de Calibración Físico:** Botón `[📍 Calibrar Físico (M18/M17)]` para liberar motores, colocar el brazo en "L" y enclavar a `(0, 120, 120)`.
* **Control de Pinza:** Apertura y cierre con `ESPACIO` o `G`.
* **Terminal G-Code y Telemetría en Vivo:** Ángulos en grados, radio actual y margen restante.

### 2. `test_hardware.py` (Diagnóstico de Consola)
Herramienta de diagnóstico rápido para verificar el hardware de forma aislada sin interfaz gráfica:
* `[1]` Probar ventilador D9 (M106 / M107).
* `[2] / [3]` Enclavar y liberar torque de motores (M17 / M18).
* `[4], [5], [6]` Prueba de movimiento individual de cada eje.
* `[7]` Apertura y cierre de pinza.
* `[8]` Test secuencial de LEDs del ULN2003 (`M3 T0`) para validar cableado.

---

## 🚀 Puesta en Marcha Rápida (Quickstart)

### Requisitos Previos (Linux / Ubuntu)
```bash
# Dependencias de Python
pip install pyserial PyQt6

# Permisos para el puerto serie
sudo usermod -aG dialout,uucp $USER
# (Reiniciar sesión si acabas de agregar los permisos)
```

### Ejecución
1. Conecta la placa por USB al ordenador (generalmente `/dev/ttyUSB0` a 9600 baud).
2. Para diagnóstico de componentes:
   ```bash
   python3 test_hardware.py
   ```
3. Para abrir la interfaz de control:
   ```bash
   python3 robot_gui.py
   ```

---

## 📋 Comandos G-Code Soportados

| Comando | Parámetros | Descripción / Acción |
| :--- | :--- | :--- |
| `G1` | `X.. Y.. Z..` | Movimiento coordinado de cinemática inversa interpolado |
| `G4` | `T<seg>` | Pausa / Dwell en segundos |
| `M3` | `T<pasos>` | **Cierra la pinza** suavemente con micropasos (ej. `M3 T600`) |
| `M3` | `T0` | **Diagnóstico de LEDs**: enciende secuencialmente LED 1 a 4 del ULN2003 |
| `M5` | `T<pasos>` | **Abre la pinza** suavemente con micropasos (ej. `M5 T600`) |
| `M17` | - | Enclava y activa corriente en todos los motores NEMA 17 |
| `M18` | - | Desactiva motores (suaves y libres para mover a mano) |
| `M106`| - | Enciende el ventilador de drivers en D9 |
| `M107`| - | Apaga el ventilador de drivers en D9 |

---

## 🗺️ Plan de Acción y Hoja de Ruta (Roadmap)

```mermaid
flowchart LR
    Fase1["1. Calibración & Garra"] --> Fase2["2. Rutinas Teach & Play"]
    Fase2 --> Fase3["3. Visión & Pick and Place"]
    Fase3 --> Fase4["4. Gemelo Digital ROS 2"]
```

* **Fase 1: Puesta a Punto y Calibración Física**
  * Corrección del cable alargador del motor 28BYJ-48 de la garra y test con `M3 T0` / `M3 T600`.
  * Verificación de repetibilidad y fijación de topes mecánicos en posición HOME.
* **Fase 2: Secuencias Automáticas (Teach & Play)**
  * Grabación interactiva de puntos desde la GUI para reproducir trayectorias cíclicas de manipulación.
  * Carga y ejecución de scripts G-Code autónomos.
* **Fase 3: Visión Artificial y *Pick & Place***
  * Integración con cámara (OpenCV) para detección de piezas por color o marcadores ArUco.
  * Calibración cámara-robot para agarre y clasificación automática sobre mesa de trabajo.
* **Fase 4: Gemelo Digital y ROS 2 (MoveIt 2 e Industria 4.0)**
  * Visualización en tiempo real del modelo URDF en RViz sincronizado con la placa física vía `robot_arm_bridge`.
  * Planificación de trayectorias en 3D con evasión de obstáculos en MoveIt 2.
  * Telemetría y control remoto vía MQTT / WebSockets.

---

## 📄 Referencias y Licencias
* Diseño mecánico original: [Florin Tobler - 3D Printable Robot Arm (Thingiverse #1718984)](https://www.thingiverse.com/thing:1718984).
* Memoria técnica y bitácora de hardware: [`docs/MEMORIA_PROYECTO.md`](docs/MEMORIA_PROYECTO.md).
* Licencia del software: [MIT License](LICENSE).
