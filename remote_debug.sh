#!/bin/bash
# Script para automatizar la depuración remota

TARGET_HOST="192.168.7.2"
TARGET_USER="spuc"
TARGET_PATH="/home/spuc/mpu"
PORT=3333
BINARY_PATH="target/armv7-unknown-linux-musleabihf/debug/examples/linux_advanced"

echo "Copiando binario al dispositivo remoto..."
scp $BINARY_PATH $TARGET_USER@$TARGET_HOST:$TARGET_PATH/

echo "Iniciando gdbserver remoto..."
ssh $TARGET_USER@$TARGET_HOST "cd $TARGET_PATH && pkill -f gdbserver || true && gdbserver :$PORT ./linux_advanced" &

# Dar tiempo para que gdbserver inicie
sleep 1
echo "GDBServer iniciado en $TARGET_HOST:$PORT"