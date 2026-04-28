cmake_minimum_required(VERSION 3.10.2)
project(actor_control_plugin)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED)

add_library(ActorControlPlugin SHARED
  src/ActorControlPlugin.cc)

target_include_directories(ActorControlPlugin PRIVATE
  /usr/include/ignition/gazebo6
  /usr/include/ignition/transport11
  /usr/include/ignition/msgs8
  /usr/include/ignition/math6
)

target_link_libraries(ActorControlPlugin
  ignition-gazebo6::ignition-gazebo6
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
  ignition-math6::ignition-math6
  ignition-plugin1::ignition-plugin1
)

target_compile_features(ActorControlPlugin PUBLIC cxx_std_17)

install(TARGETS ActorControlPlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Required so ament registers this as a ROS package and
# get_package_share_directory('actor_control_plugin') resolves correctly.
ament_package()