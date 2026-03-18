cmake_minimum_required(VERSION 3.8)
project(simulation)

# Find ament_cmake (required for ROS 2 packages)
find_package(ament_cmake REQUIRED)

# -----------------------------
# Install models and worlds
# -----------------------------
# This ensures the models and worlds are copied to the install folder
install(DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Install launch files
# -----------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Install package.xml
# -----------------------------
# So the package can be found by ament/colcon
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# ament package macros
# -----------------------------
ament_package()