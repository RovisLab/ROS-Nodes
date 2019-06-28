cmake_minimum_required(VERSION 2.8.11)
project(pioneer_gui)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  geometry_msgs
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
#  LIBRARIES client_plugin
   CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs
#  DEPENDS system_lib
)

set (RESOURCES resources.qrc)
set (PioneerGUILib_src ${PROJECT_SOURCE_DIR}/src/pioneer_gui.cpp
    ${PROJECT_SOURCE_DIR}/src/real.cpp
    ${PROJECT_SOURCE_DIR}/src/sim.cpp
    )
set (PioneerGUILib_hdr ${PROJECT_SOURCE_DIR}/include/pioneer_gui.h
    ${PROJECT_SOURCE_DIR}/include/real.h
    ${PROJECT_SOURCE_DIR}/include/sim.h)
set (PioneerGUILib_ui  ${PROJECT_SOURCE_DIR}/ui/pioneer_gui.ui
    ${PROJECT_SOURCE_DIR}/ui/real.ui
    ${PROJECT_SOURCE_DIR}/ui/sim.ui)
set (PioneerGUIBin_src ${PROJECT_SOURCE_DIR}/src/main.cpp)

qt5_add_resources(RES_SOURCES ${RESOURCES})
qt5_wrap_cpp(PioneerGUILib_hdr_moc ${PioneerGUILib_hdr})
qt5_wrap_ui (PioneerGUILib_ui_moc  ${PioneerGUILib_ui})

include_directories(
  INCLUDE_DIRS include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library (PioneerGUILib STATIC
    ${PioneerGUILib_src}
    ${PioneerGUILib_hdr_moc}
    ${PioneerGUILib_ui_moc}
)

target_link_libraries (PioneerGUILib ${catkin_LIBRARIES} yaml-cpp Qt5::Widgets Qt5::Core)
## Declare a C++ executable
add_executable(${PROJECT_NAME}_node ${PioneerGUIBin_src})
target_link_libraries(${PROJECT_NAME}_node PioneerGUILib)

