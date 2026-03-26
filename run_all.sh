#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

FRAMES_DIR="$SCRIPT_DIR/frames"
RATE_HZ="25.0"
PYTHON_BIN="/usr/bin/python3"
AUTO_BUILD="1"
START_PAUSED="1"

usage() {
  cat <<EOF
Uso:
  $0 [opciones]

Opciones:
  --frames-dir <path>   Carpeta de frames (default: $SCRIPT_DIR/frames)
  --rate-hz <float>     Frecuencia del pipeline (default: 25.0)
  --python <path>       Python para la GUI (default: /usr/bin/python3)
  --start-running       Arranca reproduciendo (default: pausado)
  --no-build            No compilar antes de ejecutar
  -h, --help            Mostrar ayuda

Ejemplo:
  $0 --frames-dir "$SCRIPT_DIR/frames" --rate-hz 20.0
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frames-dir)
      FRAMES_DIR="$2"
      shift 2
      ;;
    --rate-hz)
      RATE_HZ="$2"
      shift 2
      ;;
    --python)
      PYTHON_BIN="$2"
      shift 2
      ;;
    --start-running)
      START_PAUSED="0"
      shift
      ;;
    --no-build)
      AUTO_BUILD="0"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Opción desconocida: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ ! -d "$FRAMES_DIR" ]]; then
  echo "[ERROR] No existe la carpeta de frames: $FRAMES_DIR"
  exit 1
fi

if [[ ! -f "$SCRIPT_DIR/lane_gui.py" ]]; then
  echo "[ERROR] No se encontró lane_gui.py en: $SCRIPT_DIR"
  exit 1
fi

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "[ERROR] No existe /opt/ros/humble/setup.bash"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "$AUTO_BUILD" == "1" ]]; then
  echo "[INFO] Compilando paquete lane_detection..."
  colcon build --base-paths "$SCRIPT_DIR" --packages-select lane_detection --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  echo "[ERROR] No existe $SCRIPT_DIR/install/setup.bash"
  echo "       Ejecutá primero: colcon build --base-paths $SCRIPT_DIR --packages-select lane_detection"
  exit 1
fi

set +u
source "$SCRIPT_DIR/install/setup.bash"
set -u

NODE_PID=""
cleanup() {
  local code=$?
  if [[ -n "$NODE_PID" ]] && kill -0 "$NODE_PID" 2>/dev/null; then
    echo "[INFO] Cerrando lane_pipeline_node (PID: $NODE_PID)..."
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
  exit $code
}
trap cleanup EXIT INT TERM

echo "[INFO] Iniciando lane_pipeline_node..."
START_PAUSED_ARG="true"
if [[ "$START_PAUSED" == "0" ]]; then
  START_PAUSED_ARG="false"
fi

ros2 run lane_detection lane_pipeline_node \
  --ros-args \
  -p frames_dir:="$FRAMES_DIR" \
  -p publish_rate_hz:="$RATE_HZ" \
  -p start_paused:="$START_PAUSED_ARG" &
NODE_PID=$!

sleep 1
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "[ERROR] lane_pipeline_node no inició correctamente."
  wait "$NODE_PID" || true
  exit 1
fi

echo "[INFO] Iniciando GUI..."
exec "$PYTHON_BIN" "$SCRIPT_DIR/lane_gui.py"
