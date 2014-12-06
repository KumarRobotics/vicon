cmake_minimum_required(VERSION 2.8.3)
project(vicon_odom)

find_package(catkin REQUIRED COMPONENTS
    roscpp vicon tf2_ros
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS vicon tf2_ros
    )

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME} src/vicon_odom.cpp src/filter.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} vicon_generate_messages_cpp)

add_library(${PROJECT_NAME} src/filter.cpp src/vicon_odom_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/vicon_odom_main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
