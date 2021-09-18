cmake_minimum_required(VERSION 3.0.2)
project(scenario_editor_plugin)

add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
find_package(OpenCV REQUIRED)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)

set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
  tf
  interactive_markers
  std_msgs
  rviz
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES scenario_editor_core
#  CATKIN_DEPENDS geometry_msgs roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
include
${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_library(scenario_editor_core lib/scenario_editor_core.cpp)
add_dependencies(scenario_editor_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(scenario_editor_core
  ${catkin_LIBRARIES}
)

add_library(scenario_editor_plugin src/scenario_editor_plugin.cpp src/scenarioeditorplugin.ui)
add_dependencies(scenario_editor_plugin scenario_editor_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(scenario_editor_plugin scenario_editor_core
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
