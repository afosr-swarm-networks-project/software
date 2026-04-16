cmake_minimum_required(VERSION 3.8)
project(rf_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rf_msgs REQUIRED)

add_library(rf_world SHARED
  src/RfWorldPlugin.cc
  src/antennas/OmniAntenna.cc
  src/antennas/YagiAntenna.cc
  src/transmitters/SdfTransmitter.cc
  src/sources/WifiSource.cc
  src/sources/LoraSource.cc
  src/sources/FhssSource.cc
  src/sources/FmSource.cc
  src/sources/BluetoothSource.cc
  src/receivers/SdfReceiver.cc
  src/sinks/IqPubSink.cc
  src/channels/FreeSpaceChannel.cc
)

target_include_directories(rf_world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# gz-sim8 transitively pulls in gz-math7, gz-transport13, sdformat14
target_link_libraries(rf_world
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
)
ament_target_dependencies(rf_world rclcpp rf_msgs)

install(TARGETS rf_world
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
