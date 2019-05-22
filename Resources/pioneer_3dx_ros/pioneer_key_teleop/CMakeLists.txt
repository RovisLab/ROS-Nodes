cmake_minimum_required(VERSION 2.8.11)
project(pioneer_key_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Widgets REQUIRED)

set ( RESOURCES resources.qrc)

qt5_add_resources(RES_SOURCES ${RESOURCES})
qt5_wrap_cpp(MOC src/pioneer_key_teleop.h)
qt5_wrap_ui(UIC src/pioneer_key_teleop.ui)

catkin_package(
#  LIBRARIES client_plugin
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  INCLUDE_DIRS include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/main.cpp src/pioneer_key_teleop.cpp src/pioneer_key_teleop.h ${RES_SOURCES} src/pioneer_key_teleop.ui)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Qt5::Widgets)

