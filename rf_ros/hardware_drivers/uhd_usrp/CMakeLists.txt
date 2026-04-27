cmake_minimum_required(VERSION 3.8)
project(uhd_usrp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/usrp_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()