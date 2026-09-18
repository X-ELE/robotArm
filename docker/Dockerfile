# Imagen base oficial de ROS 2 Humble con entorno de escritorio completo
FROM osrf/ros:humble-desktop-full

# Evitar prompts interactivos durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# Instalar herramientas de desarrollo, MoveIt 2, paquetes de visión y utilidades
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-v4l2-camera \
    python3-pip \
    python3-colcon-common-extensions \
    python3-serial \
    python3-opencv \
    v4l-utils \
    nano \
    gedit \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Configurar autocompletado y entorno ROS en bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/robot_ws/ros2_ws/install/setup.bash ]; then source /root/robot_ws/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Directorio de trabajo montado desde el host
WORKDIR /root/robot_ws

CMD ["bash"]
