#!/bin/bash
# Watchdog para OFDMModulator --zmq
#
# Reinicia automáticamente en CUALQUIER salida (crash o exit limpio).
# La única forma de parar sin reinicio es Ctrl+C en este terminal.
#
# Prerequisito (una vez por sesión):
#   sudo ./scripts/isolate_cpus.bash <rango_cpu>   # ej: 0-5
#
# Uso:
#   sudo ./scripts/run_modulator.bash

if [[ $EUID -ne 0 ]]; then
    echo "[watchdog] Necesita privilegios — relanzando con sudo..."
    exec sudo "$0" "$@"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK_DIR="$(pwd)"
BINARY="${WORK_DIR}/build/OFDMModulator"
RESTART_DELAY=3

if [[ ! -x "$BINARY" ]]; then
    echo "[watchdog] Error: $BINARY no encontrado o no es ejecutable." >&2
    echo "[watchdog] Compila primero: cmake --build build --target OFDMModulator" >&2
    exit 1
fi

USER_STOPPED=0
trap 'USER_STOPPED=1' INT TERM

echo "[watchdog] ─────────────────────────────────────────────────"
echo "[watchdog]  OFDMModulator --zmq"
echo "[watchdog]  Ctrl+C en ESTE terminal  →  parada definitiva"
echo "[watchdog]  cualquier otra salida    →  reinicio en ${RESTART_DELAY}s"
echo "[watchdog] ─────────────────────────────────────────────────"
echo ""

while [[ $USER_STOPPED -eq 0 ]]; do
    "$SCRIPT_DIR/isolate_cpus.bash" run "$BINARY" --zmq
    EC=$?

    # Única condición de parada: el usuario pulsó Ctrl+C en este terminal
    [[ $USER_STOPPED -eq 1 ]] && break

    echo ""
    echo "[watchdog] Proceso terminado (exit $EC) — reiniciando en ${RESTART_DELAY}s  (Ctrl+C para abortar)"

    # Sleep interruptible: sale inmediatamente si llega SIGINT
    for (( i=0; i < RESTART_DELAY && USER_STOPPED == 0; i++ )); do sleep 1; done
done

echo "[watchdog] Detenido."
