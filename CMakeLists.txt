cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gazebo_learn)

list(APPEND CMAKE_MODULE_PATH /usr/lib64/OGRE/cmake)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hello_world SHARED example/hello_world.cpp)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(model_push SHARED example/model_push.cpp)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_library(world_edit SHARED example/world_edit.cpp)
target_link_libraries(world_edit ${GAZEBO_LIBRARIES})

add_library(system_gui SHARED example/system_gui.cpp)
target_link_libraries(system_gui ${GAZEBO_LIBRARIES})

# animated box
add_library(animated_box SHARED example/animated_box/animated_box.cpp)
target_link_libraries(animated_box ${GAZEBO_LIBRARIES})

add_executable(integrated_main example/animated_box/integrated_main.cpp)
target_link_libraries(integrated_main ${GAZEBO_LIBRARIES} pthread)

add_executable(independent_listener example/animated_box/independent_listener.cpp)
target_link_libraries(independent_listener ${GAZEBO_LIBRARIES} pthread)

# velodyne plugin
add_library(velodyne_plugin SHARED example/velodyne_plugin/velodyne_plugin.cpp)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES})

## velodyne velocity publisher
add_executable(velodyne_plugin_vel example/velodyne_plugin/velodyne_plugin_vel.cpp)
target_link_libraries(velodyne_plugin_vel ${GAZEBO_LIBRARIES})

# the project file
add_library(project_world_init SHARED src/world_init.cpp)
target_link_libraries(project_world_init ${GAZEBO_LIBRARIES})

add_library(project_robot_drive SHARED src/robot_drive.cpp)
target_link_libraries(project_robot_drive ${GAZEBO_LIBRARIES})
