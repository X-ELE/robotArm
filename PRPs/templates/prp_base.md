# PRP: [Nombre de la Característica / Feature Name]

> **Versión:** 1.0  
> **Fecha:** [Fecha de creación]  
> **Estado:** Draft | Ready | In Progress | Completed  
> **Módulo:** Core | ROS2 | Hardware | MoveIt | Vision  

---

## 1. Objetivo (Goal)
[Descripción clara y específica del estado final esperado. Qué debe funcionar en simulación y en hardware real.]

## 2. Justificación y Valor Pedagógico/Industrial (Why)
- **Impacto para los estudiantes:** [Qué concepto de robótica industrial aprenden]
- **Relación con la Industria 4.0:** [Gemelo digital, visión artificial, control distribuido, etc.]
- **Integración con el hardware:** [Cómo interactúa con la MKS Gen v1.4, motores y sensores]

## 3. Alcance y Criterios de Aceptación (Success Criteria)
- [ ] [Criterio medible 1 - Ej: El modelo 3D carga en RViz sin errores de tf]
- [ ] [Criterio medible 2 - Ej: El puente serie traduce comandos a <= 50ms de latencia]
- [ ] [Criterio medible 3 - Ej: Planificación de trayectorias libre de colisiones con MoveIt]
- [ ] [Criterio medible 4 - Ej: Detección y agarre autónomo con cámara]

---

## 4. Contexto y Referencias Necesarias

### Documentación y Recursos Clave
```yaml
Documentación Oficial:
  - url: "https://docs.ros.org/en/humble/"
    tema: "ROS 2 Humble core concepts & nodes"
  - url: "https://moveit.picknik.ai/humble/"
    tema: "MoveIt 2 motion planning & kinematics"
  - url: "https://docs.opencv.org/4.x/"
    tema: "Computer Vision & Camera calibration"

Archivos de Referencia en el Proyecto:
  - file: "Arduino/robotArm/robotArm.ino"
    porque: "Firmware base, mapeo de pines y baudrate serie (9600)"
  - file: "robot_control.py"
    porque: "Lógica probada de comunicación serie y protocolo G-Code"
  - file: "robotarm-model_files/"
    porque: "Mallas STL para eslabones y articulaciones del URDF"
```

### Arquitectura de Archivos
```bash
ros2_ws/src/
├── robot_arm_description/     # Modelado 3D, URDF, mallas STL, display launch
├── robot_arm_bridge/          # Nodo puente Python Serial hacia MKS Gen v1.4
├── robot_arm_moveit_config/   # Configuración de MoveIt 2 (SRDF, cinemática IK, colisiones)
└── robot_arm_vision/          # Nodo de visión OpenCV (detección de objetos y coordenadas)
```

### Advertencias Técnicas y Gotchas
```python
# CRITICAL GOTCHA 1: El firmware de MKS Gen v1.4 requiere retorno de carro CRLF ('\r\n') a 9600 baudios.
# CRITICAL GOTCHA 2: La placa requiere fuente externa de 12V/24V para alimentar los motores.
# CRITICAL GOTCHA 3: Coordenadas URDF en metros y radianes; firmware en milímetros y grados.
# CRITICAL GOTCHA 4: Permisos de puerto /dev/ttyUSB0 y paso de display en Docker (xhost).
```

---

## 5. Plan de Implementación (Blueprint)

### Tareas en Orden de Ejecución
```yaml
Tarea 1: [Nombre de la tarea]
  - ACCIÓN: Crear / Modificar [archivo]
  - PATRÓN: [Describir convención o clase base]
  - CRITERIO: [Cómo verificar que funciona]

Tarea 2: [Nombre de la tarea]
  ...
```

---

## 6. Ciclo de Validación (Validation Loop)

### Nivel 1: Sintaxis y Compilación
```bash
# Validar código Python
python3 -m py_compile [archivo.py]

# Compilar espacio de trabajo ROS 2
colcon build --symlink-install
source install/setup.bash
```

### Nivel 2: Verificación en Simulación / Gemelo Digital
```bash
# Lanzar visualización en RViz
ros2 launch robot_arm_description display.launch.py
```

### Nivel 3: Validación con Hardware Físico
```bash
# Conectar puente serie y verificar movimiento en banco de pruebas
ros2 run robot_arm_bridge serial_bridge_node
```

---

## 7. Lista de Verificación Final (Checklist)
- [ ] Compilación limpia sin advertencias críticas
- [ ] Visualización fluida en RViz / Gemelo digital
- [ ] Seguridad física verificada (límites de velocidad y colisión)
- [ ] Código comentado didácticamente para estudiantes
- [ ] Guía de laboratorio o documentación actualizada

---

## 8. Antipátrones a Evitar
- ❌ No enviar coordenadas cartesianas fuera del radio físico (máx. 230 mm).
- ❌ No bloquear el hilo principal de ROS con lecturas síncronas del puerto serie.
- ❌ No mover el hardware real sin antes previsualizar la trayectoria calculada en RViz/MoveIt.
- ❌ No omitir el factor de escala o unidades (ROS usa metros y radianes; Arduino usa milímetros y radianes/grados).
