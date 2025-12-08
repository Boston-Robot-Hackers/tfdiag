cmake_minimum_required(VERSION 3.5)
project(ros2diag)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Create bin directory with wrapper scripts
# These will be installed to bin/ which gets added to PATH automatically
configure_file(ros2diag/ros2diag_help.py ${CMAKE_CURRENT_BINARY_DIR}/ros2diag COPYONLY)
configure_file(ros2diag/tf_error_detector.py ${CMAKE_CURRENT_BINARY_DIR}/tf_error_detector COPYONLY)
configure_file(ros2diag/ros2_params.py ${CMAKE_CURRENT_BINARY_DIR}/ros2_params COPYONLY)
configure_file(ros2diag/net_latency.py ${CMAKE_CURRENT_BINARY_DIR}/net_latency COPYONLY)
configure_file(ros2diag/tf_lister.py ${CMAKE_CURRENT_BINARY_DIR}/tf_lister COPYONLY)
configure_file(ros2diag/tf_checker.py ${CMAKE_CURRENT_BINARY_DIR}/tf_checker COPYONLY)
configure_file(ros2diag/tf_topics.py ${CMAKE_CURRENT_BINARY_DIR}/tf_topics COPYONLY)
configure_file(ros2diag/tf_timechecker.py ${CMAKE_CURRENT_BINARY_DIR}/tf_timechecker COPYONLY)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/ros2diag
  ${CMAKE_CURRENT_BINARY_DIR}/tf_error_detector
  ${CMAKE_CURRENT_BINARY_DIR}/ros2_params
  ${CMAKE_CURRENT_BINARY_DIR}/net_latency
  ${CMAKE_CURRENT_BINARY_DIR}/tf_lister
  ${CMAKE_CURRENT_BINARY_DIR}/tf_checker
  ${CMAKE_CURRENT_BINARY_DIR}/tf_topics
  ${CMAKE_CURRENT_BINARY_DIR}/tf_timechecker
  DESTINATION bin
)

ament_package()
