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
#   ./scripts/run_full_pipeline.sh              # all three components
#   ./scripts/run_full_pipeline.sh --no-viz     # without 3D visualizer
#   ./scripts/run_full_pipeline.sh --sim-only   # only sim publisher (for manual C++ start)
#
# Prerequisites:
#   - mocap_ir_cpp built (bin/mocap_main exists)
#   - Python dependencies installed (pip install -r requirements.txt)
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"
CPP_DIR="$(dirname "$SIM_DIR")/mocap_ir_cpp"

SIM_CALIB="$SIM_DIR/config/calibration_sim.json"
MOCAP_CONFIG="$SIM_DIR/config/mocap_config.json"

NO_VIZ=false
SIM_ONLY=false

for arg in "$@"; do
    case $arg in
        --no-viz)   NO_VIZ=true ;;
        --sim-only) SIM_ONLY=true ;;
    esac
done

# Generate calibration if missing
if [ ! -f "$SIM_CALIB" ]; then
    echo "[setup] Generating simulation calibration..."
    python3 "$SIM_DIR/generate_sim_calib.py"
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
"$CPP_DIR/bin/mocap_main" \
    --zmq-host localhost \
    --calib "$SIM_CALIB" \
    --mocap-config "$MOCAP_CONFIG" \
    --zmq \
    --display &
CPP_PID=$!
PIDS+=($CPP_PID)
sleep 2

# 3. Start visualizer (optional)
if [ "$NO_VIZ" = false ]; then
    echo "[3/3] Starting 3D pose visualizer (port 5556)..."
    python3 "$CPP_DIR/tools/visualizer/pose_visualizer.py" \
        --config "$CPP_DIR/tools/visualizer/visualizer_config.json" &
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
