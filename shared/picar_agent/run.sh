#!/usr/bin/env bash
# Starts the Gazebo twin in the background and the AI driver in this terminal.
# Ctrl+C stops both. The twin's messages go to sim.log.
source /opt/ros/jazzy/setup.bash
source ~/picar_ws/install/setup.bash
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
cd "$(dirname "$(readlink -f "$0")")"

ollama stop qwen3-vl:4b-instruct 2>/dev/null
setsid ros2 launch picar_gazebo sim.launch.py > sim.log 2>&1 &
SIM=$!
trap 'kill -INT -- -$SIM 2>/dev/null; wait $SIM' EXIT

echo "Starting the twin, waiting for the camera (messages in sim.log)..."
until timeout 5 ros2 topic echo --once /camera/image_raw --field header.stamp > /dev/null 2>&1; do
    kill -0 $SIM 2>/dev/null || { echo "The twin did not start, see sim.log"; exit 1; }
    sleep 1
done

source ~/rai_venv/bin/activate
python3 agent.py