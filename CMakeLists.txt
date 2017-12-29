cmake_minimum_required(VERSION 2.8.3)
project(aurora)

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    geometry_msgs
    roscpp
    roslib
    tf
    roslaunch
)
#include_directories(include/odom_test_node ${CATKIN_INCLUDE_DIRS})
catkin_package()

#Qt packages
#set (CMAKE_PREFIX_PATH "/home/konstantin/Qt5.9/5.9.1/gcc_64")
find_package(Qt5Widgets)
find_package(Qt5Network)
include_directories(${Qt5Widgets_INCLUDE_DIRS} ${CATKIN_INCLUDE_DIRS})
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-std=c++11 -fPIC)

#file(GLOB ODOM_TEST_SRC ${CMAKE_CURRENT_SOURCE_DIR}/odom_test_node/*.cpp)

set(ODOM_TEST_SRC ${PROJECT_SOURCE_DIR}/src/odom_test_node/main.cpp
                  ${PROJECT_SOURCE_DIR}/src/odom_test_node/mainwindow.cpp
                  ${PROJECT_SOURCE_DIR}/src/odom_test_node/nodeqt.cpp
                  ${PROJECT_SOURCE_DIR}/src/odom_test_node/odometrymap.cpp
                  ${PROJECT_SOURCE_DIR}/src/odom_test_node/astar.cpp
                  ${PROJECT_SOURCE_DIR}/src/odom_test_node/pathcontroller.cpp)

add_executable(odom_test_node ${ODOM_TEST_SRC} ${QT_MOC_HPP})
target_link_libraries(odom_test_node Qt5::Widgets ${catkin_LIBRARIES})
install(TARGETS odom_test_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(ODOM_TEST_SRC ${PROJECT_SOURCE_DIR}/src/remote_control_server/main.cpp
                  ${PROJECT_SOURCE_DIR}/src/remote_control_server/nodeqt.cpp)

add_executable(remote_control_server_node ${ODOM_TEST_SRC} ${QT_MOC_HPP})
target_link_libraries(remote_control_server_node Qt5::Widgets Qt5::Network ${catkin_LIBRARIES})
install(TARGETS remote_control_server_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(odom_node src/odom_node/odom_node.cpp)

target_link_libraries(odom_node
   ${catkin_LIBRARIES}
 )

add_executable(setup_tf_node src/setup_tf_node/setup_tf_node.cpp)

target_link_libraries(setup_tf_node
   ${catkin_LIBRARIES}
)

install(TARGETS odom_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS setup_tf_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

roslaunch_add_file_check(launch)
