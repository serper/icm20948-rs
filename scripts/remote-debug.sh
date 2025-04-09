#!/bin/bash
# Script para automatizar la depuración remota

# Valores por defecto que pueden ser sobreescritos por variables de entorno
TARGET_HOST="${TARGET_HOST:-192.168.8.2}"
TARGET_USER="${TARGET_USER:-spuc}"
TARGET_PATH="${TARGET_PATH:-/home/spuc/mpu}"
PORT="${PORT:-3333}"
BINARY_NAME="${BINARY_NAME:-linux_advanced}"
BINARY_PATH="target/armv7-unknown-linux-musleabihf/debug/examples/${BINARY_NAME}"

# Verifica si se debe omitir la copia
if [ "$1" != "--skip-copy" ]; then
    echo "Copiando binario $BINARY_NAME al dispositivo remoto..."
    scp $BINARY_PATH ${TARGET_USER}@${TARGET_HOST}:${TARGET_PATH}/
else
    echo "Omitiendo copia del binario..."
fi

echo "Eliminando cualquier gdbserver existente..."
ssh ${TARGET_USER}@${TARGET_HOST} "pkill -f gdbserver || true"

echo "Iniciando gdbserver remoto para $BINARY_NAME..."
ssh ${TARGET_USER}@${TARGET_HOST} "cd ${TARGET_PATH} && gdbserver :${PORT} ./${BINARY_NAME}" &

# Dar tiempo para que gdbserver inicie
sleep 2
echo "GDBServer iniciado en ${TARGET_HOST}:${PORT}"