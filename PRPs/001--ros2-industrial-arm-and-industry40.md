# PRP 001: Integración de ROS 2, Gemelo Digital y Celda de Manufactura Industria 4.0

> **Versión:** 1.0  
> **Fecha:** 2026-09-03  
> **Estado:** Ready / In Progress  
> **Proyecto:** Brazo Robótico Educativo (MKS Gen v1.4 + ROS 2 + Visión Artificial)  
> **Nivel:** Universitario / Técnico Superior en Mecatrónica, Robótica y Automatización  

---

## 1. Objetivo General (Goal)

Transformar el brazo robótico impreso en 3D (Florin Tobler) controlado por la placa MKS Gen v1.4 en una **plataforma educativa completa de Robótica Industrial e Industria 4.0**, dotándolo de:
1. Un **Gemelo Digital 3D interactivo en ROS 2 (RViz2)** con cinemática directa e inversa precisa a partir de las mallas STL originales.
2. Un **Puente de Hardware en tiempo real** que sincronice la simulación digital con el brazo físico mediante comunicación serie robusta (G-Code a 9600 baudios).
3. Planificación avanzada de trayectorias y evasión de obstáculos con **MoveIt 2**.
4. Un sistema de **Visión Artificial con OpenCV** para automatización de procesos de clasificación y agarre (*Pick and Place*) autónomo de piezas.
5. Una suite de **Guías Pedagógicas y Laboratorios Prácticos** para que los estudiantes aprendan los estándares de la robótica industrial moderna.

---

## 2. Justificación y Valor Pedagógico/Industrial (Why)

* **Brecha Educativa:** Muchos estudiantes solo interactúan con brazos robots a través de interfaces propietarias o código Arduino básico sin cinemática real. Aprender ROS 2 y MoveIt 2 les otorga las competencias exactas que demandan las empresas automotrices, aeroespaciales y logísticas (KUKA, ABB, Fanuc, Universal Robots).
* **Seguridad y Metodología Industrial:** Enseña el concepto de **Gemelo Digital**: jamás se envía un robot a una trayectoria sin validarla previamente en simulación para prevenir colisiones costosas.
* **Paradigma Industria 4.0:** Conecta robótica con sensado visual inteligente, permitiendo que el robot tome decisiones dinámicas en base a lo que ve en la mesa de trabajo (color, forma, código ArUco/QR).

---

## 3. Criterios de Éxito (Success Criteria)

- [ ] **Hito 1 (Entorno):** Contenedor Docker con ROS 2 Humble/Jazzy con aceleración gráfica (RViz2) y acceso al puerto serie `/dev/ttyUSB0` operativo en un solo comando.
- [ ] **Hito 2 (URDF y Cinemática):** Paquete `robot_arm_description` con el modelo URDF/Xacro del brazo completo usando los STL de `robotarm-model_files/`, con límites articulares físicos correctos.
- [ ] **Hito 3 (Gemelo Digital en RViz2):** Visualización interactiva donde los estudiantes puedan mover sliders (`joint_state_publisher_gui`) y ver cómo se articula el robot en pantalla 3D.
- [ ] **Hito 4 (Puente Físico Serie):** Nodo `robot_arm_bridge` en Python que traduce `/joint_states` a comandos G-Code y mueve los motores de la MKS Gen v1.4 con latencia menor a 100 ms.
- [ ] **Hito 5 (MoveIt 2 Industrial):** Configuración de MoveIt 2 que permita arrastrar el efector con una esfera 3D interactiva y planificar movimientos libres de colisión con la mesa de trabajo.
- [ ] **Hito 6 (Visión Artificial Pick & Place):** Nodo de visión OpenCV que identifique objetos en la mesa de trabajo, calcule sus coordenadas en el espacio del robot y ejecute una rutina autónoma de agarre y clasificación.
- [ ] **Hito 7 (Documentación Didáctica):** Guía completa de laboratorio paso a paso con explicaciones matemáticas y teóricas para docentes y alumnos.

---

## 4. Contexto y Arquitectura Técnica

### Estructura de Paquetes en `ros2_ws/src/`
```bash
ros2_ws/src/
├── robot_arm_description/         # Definición URDF, mallas STL, archivos Xacro y launch de visualización
│   ├── meshes/                    # Copia optimizada de mallas STL para visual y colisión
│   ├── urdf/                      # robot_arm.urdf.xacro (árbol cinemático de links y joints)
│   ├── rviz/                      # Configuración de RViz2 guardada (perspectiva, rejilla, TF)
│   └── launch/                    # display.launch.py (lanza RViz2 + joint_state_publisher_gui)
├── robot_arm_bridge/              # Nodo de enlace entre tópicos ROS 2 y puerto serie MKS Gen v1.4
│   ├── robot_arm_bridge/
│   │   ├── __init__.py
│   │   └── serial_bridge_node.py  # Suscriptor de /joint_states -> envía G1 a /dev/ttyUSB0
│   └── launch/
│       └── hardware_bridge.launch.py
├── robot_arm_moveit_config/       # Configuración generada con MoveIt Setup Assistant
│   ├── config/                    # SRDF, cinemática KDL/IKFast, límites de aceleración
│   └── launch/                    # move_group.launch.py, demo.launch.py
└── robot_arm_vision/              # Nodo de visión artificial para Industria 4.0
    ├── robot_arm_vision/
    │   ├── __init__.py
    │   ├── camera_calibrator.py   # Calibración de cámara intrínseca y extrínseca (matriz homográfica)
    │   └── object_detector.py     # Detección por color / ArUco -> Publica objetivo en /target_pose
    └── launch/
        └── vision_pick_and_place.launch.py
```

### Tabla Cinemática de Articulaciones (Links & Joints)
| Articulación (Joint) | Tipo | Eje de Rotación | Límites Físicos | Conexión en MKS Gen |
| :--- | :--- | :--- | :--- | :--- |
| `joint_base_rotate` | Revoluta | Z $[0, 0, 1]$ | $-90^\circ$ a $+90^\circ$ | Z Driver (`stepperRotate`) |
| `joint_lower_arm` | Revoluta | X $[1, 0, 0]$ | $-30^\circ$ a $+85^\circ$ | Y Driver (`stepperLower`, hombro) |
| `joint_upper_arm` | Revoluta | X $[1, 0, 0]$ | $-110^\circ$ a $+45^\circ$| X Driver (`stepperHigher`, codo) |
| `joint_wrist_rotate`| Revoluta | Y $[0, 1, 0]$ | $-180^\circ$ a $+180^\circ$| E0 Driver (`stepperExtruder`, muñeca)|
| `joint_gripper` | Prismática / Revoluta | X | $0$ a $30\text{ mm}$ | AUX-1 (`stepper`, 28BYJ-48) |

### Puntos Críticos y Gotchas Detectados
1. **Unidades de Medida:** ROS 2 trabaja estrictamente en **metros** y **radianes**; el firmware de la placa Arduino trabaja en **milímetros** y **grados/pasos**. El nodo puente debe aplicar la conversión exacta:
   $$\text{mm} = \text{metros} \times 1000.0, \quad \text{grados} = \text{rad} \times \frac{180.0}{\pi}$$
2. **Protocolo Serie:** La placa MKS Gen v1.4 con el firmware corregido requiere comandos terminados en retorno de carro `\r\n` a **9600 baudios**.
3. **Flujo de Control en Hardware:** Para evitar sobrecargar el búfer de la Arduino Mega, el nodo puente no debe enviar ráfagas continuas de 100 Hz; debe limitar la tasa de actualización a 10–20 Hz o esperar la confirmación `ok` de la placa.
4. **Alimentación:** Recordar siempre a los alumnos que el USB solo alimenta la lógica ATmega2560 (5V); la fuente de 12V/24V es mandatoria para los motores.

---

## 5. Fases del Plan de Implementación (Roadmap)

```
[FASE 1: Contenedor Docker ROS 2] 
  │
  ▼
[FASE 2: Modelo URDF/Xacro con Mallas STL]
  │
  ▼
[FASE 3: Gemelo Digital Interactivo en RViz2]
  │
  ▼
[FASE 4: Nodo Puente Serie PC <-> MKS Gen v1.4]
  │
  ▼
[FASE 5: Planificación Industrial con MoveIt 2]
  │
  ▼
[FASE 6: Visión Artificial y Pick & Place Autónomo]
```

### Fase 1: Entorno de Trabajo Contenerizado (Docker)
- Crear `docker/Dockerfile` con imagen base Ubuntu 22.04 + ROS 2 Humble Desktop Full + MoveIt 2 + OpenCV + dependencias de compilación (`colcon`, `rosdep`).
- Crear script `docker/start_ros.sh` con passthrough de pantalla gráfica (X11/Wayland vía `/tmp/.X11-unix`) y mapeo directo del puerto serie `/dev/ttyUSB0` con permisos de grupo `dialout`.
- Montar la carpeta del proyecto `/home/lms/Proyectos/robotArm` como volumen interno `/root/robot_ws`.

### Fase 2: Paquete `robot_arm_description` y URDF
- Organizar las mallas STL visuales en `ros2_ws/src/robot_arm_description/meshes/`:
  - `base.stl`, `socket.stl`, `lowerShank.stl`, `upperShank.stl`, `triplate.stl`, `manipulator.stl`, `gripperBase.stl`.
- Escribir `urdf/robot_arm.urdf.xacro` con la cinemática de eslabones de 120 mm cada uno, inercias estimadas y límites angulares seguros.
- Crear `launch/display.launch.py` para visualizar el modelo en RViz2 con `robot_state_publisher` y `joint_state_publisher_gui`.

### Fase 3: Pruebas del Gemelo Digital
- Validar que al mover los sliders de cada eje en la interfaz gráfica de ROS 2, los eslabones giren exactamente sobre sus ejes físicos correspondientes sin desfasajes ni traslapes.

### Fase 4: Nodo Puente de Hardware (`robot_arm_bridge`)
- Crear paquete `robot_arm_bridge` con nodo `serial_bridge_node.py`:
  - Suscripción al tópico `/joint_states`.
  - Cinemática directa e inversa para verificar coordenadas cartesianas $(X, Y, Z)$.
  - Envío automático de `M17` al conectar.
  - Envío de comandos `G1 X... Y... Z... E...` al puerto serie `/dev/ttyUSB0`.
  - Modo simulación / hardware mediante parámetro ROS `use_hardware: true/false`.

### Fase 5: Configuración de MoveIt 2
- Generar el paquete `robot_arm_moveit_config`:
  - Definición de grupos de planificación: `arm` (articulaciones de base, hombro, codo, muñeca) y `gripper` (pinza).
  - Configuración de estados predefinidos (*Poses*): `home`, `rest`, `ready_pick`, `drop_zone`.
  - Matriz de colisiones (`ACM - Allowed Collision Matrix`) para permitir movimiento natural entre engranajes adyacentes pero evitar colisión contra la mesa de trabajo o la base.

### Fase 6: Módulo de Visión Artificial e Industria 4.0
- Crear paquete `robot_arm_vision`:
  - Nodo para capturar video desde cámara web USB (`/dev/video0`).
  - Detección de piezas por rango de color HSV (piezas rojas, azules, verdes) o marcadores ArUco.
  - Transformada de coordenadas cámara $\rightarrow$ base del robot (Calibración ojo-a-mano / *Eye-to-Hand*).
  - Secuencia de control para automatizar la celda: Detectar $\rightarrow$ Calcular $(X, Y, Z)$ $\rightarrow$ Planificar con MoveIt $\rightarrow$ Bajar $\rightarrow$ Cerrar pinza $\rightarrow$ Subir $\rightarrow$ Clasificar en zona designada.

---

## 6. Ciclo de Validación (Validation Loop)

1. **Validación Sintáctica y Compilación:**
   ```bash
   colcon build --symlink-install --packages-select robot_arm_description robot_arm_bridge
   source install/setup.bash
   ```
2. **Validación del Modelo URDF (Chequeo de Integridad):**
   ```bash
   check_urdf <(xacro src/robot_arm_description/urdf/robot_arm.urdf.xacro)
   ```
3. **Validación en Gemelo Digital:**
   ```bash
   ros2 launch robot_arm_description display.launch.py
   ```
4. **Validación del Puente Físico Serie:**
   ```bash
   ros2 launch robot_arm_bridge hardware_bridge.launch.py port:=/dev/ttyUSB0
   ```
5. **Validación de Planificación MoveIt 2:**
   ```bash
   ros2 launch robot_arm_moveit_config demo.launch.py
   ```

---

## 7. Lista de Verificación Final de Calidad
- [ ] Contenedor Docker inicia con aceleración visual sin errores de pantalla.
- [ ] URDF refleja exactamente las dimensiones reales (brazos de 120 mm).
- [ ] No existen oscilaciones numéricas ni singularidades en los cálculos cinemáticos.
- [ ] Tasa de refresco serie acoplada a la capacidad de procesamiento de la MKS Gen v1.4.
- [ ] Código y comentarios en español y alineados con las convenciones PEP8 / ROS2 C++.
