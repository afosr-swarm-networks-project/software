cmake_minimum_required(VERSION 3.8)
project(rf_signal_proc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/stft_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
