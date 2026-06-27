cmake_minimum_required(VERSION 3.5)
project(saat_core)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# ── Install Python package ────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# ── Install individual node scripts as executables ───────────────────────────
install(PROGRAMS
  saat_core/frame_capture_node.py
  saat_core/frame_divider_node.py
  saat_core/frame_speed_divider_node.py
  saat_core/volume_divider_node.py
  saat_core/vision_init_node.py
  saat_core/vision_node.py
  saat_core/action_node.py
  saat_core/data_collector_node.py
  saat_core/infection_description_node.py
  saat_core/conv_speed_node.py
  saat_core/servo_speed_node.py
  saat_core/main_speed_node.py
  saat_core/speed_publisher_node.py
  saat_core/area_node.py
  saat_core/volume_estimation_node.py
  saat_core/mass_estimation_node.py
  saat_core/data_collection_node.py
  saat_core/webpage_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch and config files ──────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
