cmake_minimum_required(VERSION 3.8)
project(rf_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(rf_pipeline)

install(PROGRAMS
  scripts/stft_node
  scripts/yolo_detector
  scripts/scoreboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES resource/best.pt
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
