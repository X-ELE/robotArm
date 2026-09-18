# Guía Maestra: Brazo Robótico Industrial con ROS 2, Gemelo Digital y Visión Artificial (Industria 4.0)

> **Destinatarios:** Estudiantes, docentes y profesionales de Ingeniería Mecatrónica, Robótica, Electrónica y Automatización.  
> **Plataforma:** Brazo Robótico 3D (Florin Tobler) + MKS Gen v1.4 (ATmega2560) + ROS 2 Humble + MoveIt 2 + OpenCV.  

---

## Índice de Contenidos

1. [Visión General de la Celda de Manufactura](#1-visión-general-de-la-celda-de-manufactura)
2. [Arquitectura del Sistema: Del Gemelo Digital al Hardware](#2-arquitectura-del-sistema-del-gemelo-digital-al-hardware)
3. [Módulo 1: Puesta en Marcha del Entorno Contenerizado (Docker + ROS 2)](#3-módulo-1-puesta-en-marcha-del-entorno-contenerizado-docker--ros-2)
4. [Módulo 2: Modelado URDF y Gemelo Digital en RViz2](#4-módulo-2-modelado-urdf-y-gemelo-digital-en-rviz2)
5. [Módulo 3: El Puente de Hardware Serie (ROS 2 ↔ MKS Gen v1.4)](#5-módulo-3-el-puente-de-hardware-serie-ros-2--mks-gen-v14)
6. [Módulo 4: Planificación de Movimientos Industriales con MoveIt 2](#6-módulo-4-planificación-de-movimientos-industriales-con-moveit-2)
7. [Módulo 5: Visión Artificial y Automatización Pick & Place (Industria 4.0)](#7-módulo-5-visión-artificial-y-automatización-pick--place-industria-40)
8. [Laboratorios Prácticos Sugeridos para Evaluación](#8-laboratorios-prácticos-sugeridos-para-evaluación)

---

## 1. Visión General de la Celda de Manufactura

En la industria automotriz y de manufactura avanzada, los brazos robóticos (KUKA, ABB, Fanuc, Universal Robots) no se programan de manera aislada ni con movimientos ciegos paso a paso. Se integran dentro de **Celdas de Manufactura Inteligentes** bajo el paradigma de la **Industria 4.0**:

* **Simulación Previa (Gemelo Digital):** Toda trayectoria se planifica y valida virtualmente en 3D para garantizar que no haya colisiones contra la mesa, herramientas o piezas circundantes.
* **Cinemática Inversa en Tiempo Real:** El operador o algoritmo indica *"lleva la herramienta a $(X: 150, Y: 80, Z: 50)$"*, y el sistema resuelve algebraicamente a qué ángulo debe girar cada motor.
* **Percepción Sensorial (Visión Artificial):** Mediante cámaras industriales, el robot "ve" qué pieza ha llegado a la cinta transportadora, identifica su orientación y color, y ejecuta el agarre autónomo (*Pick and Place*).

---

## 2. Arquitectura del Sistema: Del Gemelo Digital al Hardware

El flujo de información se organiza en tres capas jerárquicas:

```
┌────────────────────────────────────────────────────────────────────────┐
│  CAPA SUPERIOR: TOMA DE DECISIONES Y PLANIFICACIÓN                     │
│  - Nodo de Visión OpenCV: Detecta objeto -> Coordenadas (X, Y, Z)       │
│  - MoveIt 2: Planifica trayectoria suave y evita colisiones en RViz2    │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Tópico ROS 2: /joint_states
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│  CAPA INTERMEDIA: PUENTE DE HARDWARE (ROS 2 NODE)                      │
│  - Script Python: robot_arm_bridge/serial_bridge_node.py               │
│  - Convierte radianes/metros a G-Code cartesiano (G1 X... Y... Z...)    │
│  - Control de tasa (15 Hz) para no saturar el buffer del micro         │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Cable USB (/dev/ttyUSB0 @ 9600 baud)
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│  CAPA FÍSICA: ELECTRÓNICA Y POTENCIA                                   │
│  - MKS Gen v1.4 (ATmega2560): Genera pulsos STEP/DIR en microsegundos  │
│  - Drivers A4988 / DRV8825 (alimentados con fuente 12V/24V)           │
│  - Motores NEMA 17 y reducciones mecánicas 9:32                        │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Módulo 1: Puesta en Marcha del Entorno Contenerizado (Docker + ROS 2)

Para evitar que los alumnos tengan problemas de incompatibilidad de librerías en sus computadoras, el proyecto cuenta con un entorno preconfigurado en **Docker**.

### Paso 1: Requisitos Previos en la Máquina Host (Linux)
Asegurarse de tener Docker instalado y que el usuario pertenezca al grupo `docker`:
```bash
docker --version
```

### Paso 2: Iniciar el Entorno con un Solo Comando
En la raíz del proyecto, ejecuta:
```bash
./docker/start_ros.sh
```

**¿Qué hace este script automáticamente?**
1. Habilita los permisos gráficos X11 locales (`xhost +local:root`) para que las ventanas 3D de RViz2 se abran en tu pantalla.
2. Mapea el puerto serie `/dev/ttyUSB0` hacia el contenedor con permisos completos de lectura/escritura.
3. Monta todo el código del proyecto en el directorio `/root/robot_ws`.
4. Carga las variables de entorno de ROS 2 Humble.

---

## 4. Módulo 2: Modelado URDF y Gemelo Digital en RViz2

El **URDF** (*Unified Robot Description Format*) es un archivo XML/Xacro que describe la anatomía física del robot: eslabones (`links`) y articulaciones (`joints`).

El modelo se encuentra en:
```
ros2_ws/src/robot_arm_description/urdf/robot_arm.urdf.xacro
```

### Conceptos Clave del Modelo:
1. **Links:** Representan los cuerpos rígidos. Cada uno tiene:
   - `<visual>`: La malla 3D STL que vemos en pantalla (`base.stl`, `lowerShank.stl`, etc.).
   - `<collision>`: Formas geométricas simplificadas (cilindros y cajas) que MoveIt utiliza para calcular colisiones con alto rendimiento matemático.
   - `<inertial>`: Masa y momentos de inercia para cálculos dinámicos.
2. **Joints:** Las uniones mecánicas:
   - `joint_base_rotate` (Eje Z, rotación $\pm 90^\circ$).
   - `joint_lower_arm` (Eje X, hombro con longitud de 120 mm).
   - `joint_upper_arm` (Eje X, codo con longitud de 120 mm).
   - `joint_wrist_rotate` (Eje Y, giro de muñeca).
   - `joint_gripper_left` y `joint_gripper_right` (Apertura y cierre de pinza).

### Práctica 1: Lanzar el Gemelo Digital en RViz2
Dentro del contenedor ROS 2 (o en una terminal con ROS 2 activo):

```bash
cd /root/robot_ws/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Lanzar visualización interactiva
ros2 launch robot_arm_description display.launch.py
```

*Se abrirá RViz2 junto a una pequeña ventana con barras deslizantes (`joint_state_publisher_gui`). Al mover cada slider, el estudiante observará cómo rota cada eslabón en 3D exactamente sobre su eje físico.*

---

## 5. Módulo 3: El Puente de Hardware Serie (ROS 2 ↔ MKS Gen v1.4)

El paquete `robot_arm_bridge` actúa como el intérprete entre el mundo de tópicos de ROS 2 y el microcontrolador de la placa MKS Gen v1.4.

### Cómo Funciona el Nodo `serial_bridge_node.py`:
1. Se suscribe al tópico `/joint_states`.
2. Extrae los ángulos en radianes $(\theta_{rot}, \theta_{low}, \theta_{high}, \theta_{wrist})$.
3. Calcula la **cinemática directa**:
   $$r_{side} = 2 \cdot L \cdot \sin\left(\frac{|\theta_{high}|}{2}\right), \quad (L = 120\text{ mm})$$
   $$X = r_{side} \cdot \cos(\theta_{low}) \cdot \sin(\theta_{rot})$$
   $$Y = r_{side} \cdot \cos(\theta_{low}) \cdot \cos(\theta_{rot})$$
   $$Z = r_{side} \cdot \sin(\theta_{low}) + \text{altura\_base}$$
4. Envía a la placa por USB el comando G-Code:
   ```text
   G1 X... Y... Z... E...\r\n
   ```
5. Al iniciar activa los motores con `M17` y al cerrarse los desenergiza con `M18` para seguridad.

### Práctica 2: Enlazar el Gemelo Digital con el Brazo Físico
1. En una terminal, ejecuta el visualizador:
   ```bash
   ros2 launch robot_arm_description display.launch.py
   ```
2. En otra terminal (dentro del contenedor):
   ```bash
   ros2 run robot_arm_bridge serial_bridge_node
   ```
*Ahora, al mover los controles en pantalla, el brazo físico replicará con suavidad los movimientos del gemelo digital.*

---

## 6. Módulo 4: Planificación de Movimientos Industriales con MoveIt 2

**MoveIt 2** es el estándar de oro en robótica industrial. En lugar de mover articulación por articulación, MoveIt permite mover el efector final en el espacio cartesiano.

### Capacidades que Aprenderán los Alumnos:
* **Cinemática Inversa (IK Solver):** MoveIt calcula automáticamente la combinación de ángulos para alcanzar cualquier posición $(X, Y, Z, \text{Roll}, \text{Pitch}, \text{Yaw})$.
* **Planificadores OMPL (RRT-Connect, PRM):** Generan curvas de movimiento que rodean obstáculos sin tropezar.
* **Escena de Planificación (Planning Scene):** Se puede añadir una mesa virtual, cajas o paredes. Si el alumno le pide al robot ir a un punto atravesando la mesa, MoveIt detectará la colisión, rechazará la trayectoria y buscará un camino alternativo elevado.

---

## 7. Módulo 5: Visión Artificial y Automatización Pick & Place (Industria 4.0)

En la Industria 4.0, los robots no repiten trayectorias a ciegas: se adaptan a la posición real de los objetos mediante visión por computadora.

```
       [ Cámara USB Cenital ]
                 │ (Fotografía la mesa de trabajo)
                 ▼
       [ Nodo de Visión OpenCV ]
                 │ Identifica centroide del objeto: (u, v) en píxeles
                 │ Transforma píxeles a coordenadas de la mesa: (X, Y, Z) en mm
                 ▼
       [ Cliente MoveIt 2 en Python ]
                 │ 1. Mover sobre el objeto (Z = 120 mm)
                 │ 2. Bajar a nivel de agarre (Z = 45 mm)
                 │ 3. Cerrar pinza (M3 T50)
                 │ 4. Elevar pieza (Z = 120 mm)
                 │ 5. Trasladar a zona de clasificación (Roja / Azul / Verde)
                 │ 6. Abrir pinza (M5 T50)
                 ▼
           [ Ciclo Completado ]
```

### Calibración Ojo-a-Mano (*Eye-to-Hand Calibration*):
Para transformar las coordenadas de los píxeles de la cámara $(u, v)$ a milímetros del robot $(X_{robot}, Y_{robot})$, los alumnos aplican una matriz de transformación afín u **homografía 2D**:
$$\begin{bmatrix} X_{robot} \\ Y_{robot} \\ 1 \end{bmatrix} = \mathbf{H} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$$
Basta con colocar 4 puntos de referencia en la mesa para calcular la matriz $\mathbf{H}$ con `cv2.findHomography()`.

---

## 8. Laboratorios Prácticos Sugeridos para Evaluación

### Laboratorio 1: Calibración y Modelado Cinemático
* **Objetivo:** Comprender la estructura de eslabones y verificar el gemelo digital.
* **Entregable:** El alumno modifica el archivo URDF para agregar una mesa de trabajo virtual en RViz y comprueba los límites articulares para evitar que el codo golpee la base.

### Laboratorio 2: Programación de Trayectorias Automatizadas en Python
* **Objetivo:** Diseñar una rutina secuencial de paletizado (apilar 3 cubos).
* **Entregable:** Script de ROS 2 que ordene al brazo recorrer una cuadrícula de $3 \times 3$ puntos cartesianos a velocidad constante.

### Laboratorio 3: Celda de Clasificación Inteligente Industria 4.0
* **Objetivo:** Integrar cámara web y clasificación autónoma por color.
* **Entregable:** Colocar objetos rojos y azules al azar sobre la mesa; el robot debe detectar cada pieza, recogerla y depositarla en la bandeja correspondiente según su color.
