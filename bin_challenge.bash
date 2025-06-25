#!/bin/bash

PY_SCRIPTS=(
  "door_handle.py"
  "emergency_button.py"
  "barcode_scanner.py"
  "hmi.py"
)

source ~/ros2_ws/install/setup.bash

for script in "${PY_SCRIPTS[@]}"
do
  echo "Starting $script ..."
  ros2 run BinPickingCellControl "$script" &
done

echo "All scripts started in background."

wait
