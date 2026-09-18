#!/usr/bin/env bash
set -e

# Permitir que aplicaciones dentro de Docker dibujen en la pantalla gráfica local
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
fi

echo "============================================================"
echo "  Iniciando Entorno ROS 2 Humble para Brazo Robótico        "
echo "============================================================"

# Verificar si la placa MKS Gen v1.4 está conectada
if [ -e "/dev/ttyUSB0" ]; then
    echo "  [OK] Placa MKS Gen v1.4 detectada en /dev/ttyUSB0"
else
    echo "  [AVISO] /dev/ttyUSB0 no conectado. (Modo simulación disponible)"
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

# Construir si es necesario e iniciar sesión interactiva en el contenedor
docker compose build
docker compose run --rm ros2_dev
