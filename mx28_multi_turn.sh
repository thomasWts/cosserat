ros2 run dxl_python_demo mx28_raw_sync_node \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud_rate:=57600 \
  -p dxl_ids:="[1, 2, 3, 4, 5, 6]" \
  -p configure_multiturn:=false \
  -p resolution_divider:=1 \
  -p min_raw:=-28672 \
  -p max_raw:=28672
