#!/bin/bash

TARGET=armv7-unknown-linux-musleabihf
REMOTE_USER=spuc
REMOTE_HOST=192.168.8.2
REMOTE_PATH=/home/spuc/debug
REMOTE_BIN=$1
LOCAL_BIN=target/$TARGET/debug/examples/$REMOTE_BIN
GDB_PORT=3333

echo "[1] Compilando binario..."
cargo build --target=$TARGET

if [ ! -f "$LOCAL_BIN" ]; then
    echo "❌ No se encontró el binario $LOCAL_BIN"
    exit 1
fi

echo "[2] Subiendo binario a $REMOTE_HOST:$REMOTE_PATH..."
ssh $REMOTE_USER@$REMOTE_HOST "mkdir -p $REMOTE_PATH"
if [ "$2" != "--skip-copy" ]; then
    scp $LOCAL_BIN $REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/
fi

echo "[3] Iniciando gdbserver en $REMOTE_HOST..."
ssh $REMOTE_USER@$REMOTE_HOST "
    pkill gdbserver 2>/dev/null;
    cd $REMOTE_PATH && chmod +x $REMOTE_BIN && gdbserver :$GDB_PORT ./$REMOTE_BIN
"
