cmake_minimum_required(VERSION 3.10)
project(xarm5_description)

# No code to build — just install resources.
find_package(ament_cmake REQUIRED)

# Install runtime assets so launch can find them via ament index
install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
