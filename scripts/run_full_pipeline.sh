#!/bin/bash
#
# Mocap Simulation Full Pipeline Launcher
#
# Starts all three components in the correct order:
#   1. sim_publisher.py   – generates IR images, publishes via ZMQ (ports 5552/5553)
#   2. mocap_main (C++)   – receives images, runs vision pipeline, publishes poses (port 5556)
#   3. pose_visualizer.py – subscribes to pose data and displays 3D visualization
#
# Usage:
#   ./scripts/run_full_pipeline.sh
#   ./scripts/run_full_pipeline.sh /path/to/mocap_config_runtime.json
#   ./scripts/run_full_pipeline.sh --mocap-config /path/to/mocap_config_runtime.json
#   ./scripts/run_full_pipeline.sh --mocap-config /path/to/mocap_config_runtime.json --no-viz
#   ./scripts/run_full_pipeline.sh --sim-only   # only sim publisher (for manual C++ start)
#
# Prerequisites:
#   - mocap_ir_cpp built (bin/mocap_main exists)
#   - Python dependencies installed (pip install -r requirements.txt)
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"
CPP_DIR="$(dirname "$SIM_DIR")/mocap_ir_cpp"

DEFAULT_SIM_CALIB="$SIM_DIR/config/calibration_sim.json"
SIM_CALIB="$DEFAULT_SIM_CALIB"
MOCAP_CONFIG="$SIM_DIR/config/mocap_config.json"

NO_VIZ=false
SIM_ONLY=false

usage() {
    cat <<EOF
Usage:
  $(basename "$0") [mocap_config_path] [options]

Options:
  -c, --mocap-config PATH   Mocap config JSON used by sim_publisher and mocap_main
      --calib PATH          Simulation calibration JSON
      --no-viz             Skip the visualizer
      --sim-only           Start only sim_publisher
  -h, --help               Show this help

Examples:
  $(basename "$0")
  $(basename "$0") /home/zm/mocap_ir/mocap_ir_all/mocap_config_runtime.json
  $(basename "$0") --mocap-config /home/zm/mocap_ir/mocap_ir_all/mocap_config_runtime.json
EOF
}

resolve_path() {
    local path="$1"
    if [[ "$path" = /* ]]; then
        printf '%s\n' "$path"
    else
        printf '%s\n' "$PWD/$path"
    fi
}

POSITIONAL_CONFIG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--mocap-config)
            if [[ $# -lt 2 ]]; then
                echo "[ERROR] Missing value for $1" >&2
                usage
                exit 1
            fi
            MOCAP_CONFIG="$(resolve_path "$2")"
            shift 2
            ;;
        --calib)
            if [[ $# -lt 2 ]]; then
                echo "[ERROR] Missing value for $1" >&2
                usage
                exit 1
            fi
            SIM_CALIB="$(resolve_path "$2")"
            shift 2
            ;;
        --no-viz)
            NO_VIZ=true
            shift
            ;;
        --sim-only)
            SIM_ONLY=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "[ERROR] Unknown option: $1" >&2
            usage
            exit 1
            ;;
        *)
            if [[ -n "$POSITIONAL_CONFIG" ]]; then
                echo "[ERROR] Only one mocap config path may be provided." >&2
                usage
                exit 1
            fi
            POSITIONAL_CONFIG="$(resolve_path "$1")"
            shift
            ;;
    esac
done

if [[ -n "$POSITIONAL_CONFIG" ]]; then
    MOCAP_CONFIG="$POSITIONAL_CONFIG"
fi

if [ ! -f "$MOCAP_CONFIG" ]; then
    echo "[ERROR] Mocap config not found: $MOCAP_CONFIG" >&2
    exit 1
fi

# Generate calibration if missing
if [ "$SIM_CALIB" = "$DEFAULT_SIM_CALIB" ] && [ ! -f "$SIM_CALIB" ]; then
    echo "[setup] Generating simulation calibration..."
    python3 "$SIM_DIR/generate_sim_calib.py"
fi

if [ ! -f "$SIM_CALIB" ]; then
    echo "[ERROR] Calibration file not found: $SIM_CALIB" >&2
    exit 1
fi

# Auto-detect: if C++ binary missing, fallback to sim-only mode
if [ "$SIM_ONLY" = false ] && [ ! -f "$CPP_DIR/bin/mocap_main" ]; then
    echo "[WARNING] $CPP_DIR/bin/mocap_main not found."
    echo "          Falling back to --sim-only mode."
    echo "          Build mocap_ir_cpp first for full pipeline."
    echo ""
    SIM_ONLY=true
fi

PIDS=()

cleanup() {
    echo ""
    echo "[shutdown] Stopping all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "[shutdown] Done."
}
trap cleanup EXIT INT TERM

echo "============================================"
echo "  Mocap Simulation Full Pipeline"
echo "============================================"
echo ""
echo "Mocap config : $MOCAP_CONFIG"
echo "Calibration  : $SIM_CALIB"
echo ""

# 1. Start sim publisher
echo "[1/3] Starting sim publisher (ports 5552/5553)..."
cd "$SIM_DIR"
python3 sim_publisher.py \
    -c "$MOCAP_CONFIG" \
    --calib "$SIM_CALIB" &
SIM_PID=$!
PIDS+=($SIM_PID)
sleep 2

if [ "$SIM_ONLY" = true ]; then
    echo ""
    echo "Sim publisher running (PID $SIM_PID)."
    echo ""
    echo "To start C++ pipeline manually, run in another terminal:"
    echo "  cd $CPP_DIR"
    echo "  ./bin/mocap_main --zmq-host localhost \\"
    echo "      --calib $SIM_CALIB \\"
    echo "      --mocap-config $MOCAP_CONFIG \\"
    echo "      --zmq --display"
    echo ""
    wait $SIM_PID
    exit 0
fi

# 2. Start C++ pipeline
echo "[2/3] Starting C++ pipeline (zmq-host=localhost, zmq pose on 5556)..."
(
    cd "$CPP_DIR" || exit 1
    ./bin/mocap_main \
        --zmq-host localhost \
        --calib "$SIM_CALIB" \
        --mocap-config "$MOCAP_CONFIG" \
        --zmq \
        --display
) &
CPP_PID=$!
PIDS+=($CPP_PID)
sleep 2

# 3. Start visualizer (optional)
if [ "$NO_VIZ" = false ]; then
    echo "[3/3] Starting 3D pose visualizer (port 5556)..."
    "$CPP_DIR/scripts/run_visualizer.sh" &
    VIZ_PID=$!
    PIDS+=($VIZ_PID)
else
    echo "[3/3] Visualizer skipped (--no-viz)."
fi

echo ""
echo "All components running. Press Ctrl+C to stop."
echo "  sim_publisher : PID $SIM_PID"
[ -n "$CPP_PID" ] && echo "  mocap_main    : PID $CPP_PID"
[ -n "$VIZ_PID" ] && echo "  visualizer    : PID $VIZ_PID"
echo ""

wait
