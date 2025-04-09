# Script para automatizar la depuración remota en Windows
param (
    [switch]$SkipCopy = $false
)

$TARGET_HOST = "192.168.7.2"
$TARGET_USER = "spuc"
$TARGET_PATH = "/home/spuc/mpu"
$PORT = 3333
$BINARY_PATH = "target/armv7-unknown-linux-musleabihf/debug/examples/linux_advanced"

if (-not $SkipCopy) {
    Write-Host "Copiando binario al dispositivo remoto..."
    scp $BINARY_PATH "${TARGET_USER}@${TARGET_HOST}:${TARGET_PATH}/"
}
else {
    Write-Host "Omitiendo copia del binario..."
}

Write-Host "Eliminando cualquier gdbserver existente..."
ssh "${TARGET_USER}@${TARGET_HOST}" "pkill -f gdbserver; exit 0"

Write-Host "Iniciando gdbserver remoto..."
Start-Process -NoNewWindow ssh -ArgumentList "${TARGET_USER}@${TARGET_HOST}", "cd ${TARGET_PATH} && gdbserver :${PORT} ./linux_advanced"

Write-Host "GDBServer iniciado en ${TARGET_HOST}:${PORT}"
Start-Sleep -Seconds 2