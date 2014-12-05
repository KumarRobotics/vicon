cmake_minimum_required(VERSION 2.8.3)
project(vicon)

find_package(catkin REQUIRED COMPONENTS
    roscpp geometry_msgs nav_msgs std_srvs
    message_generation cmake_modules
    )
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# =============================
# External Project vicon_driver
# =============================
set(VICON_DRIVER vicon_driver)
include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${VICON_DRIVER})
ExternalProject_Add(${VICON_DRIVER}
    UPDATE_COMMAND ""
    URL /home/chao/Workspace/repo/kumar_robotics/vicon/vicon_driver
    CMAKE_ARGS -DBUILD_SHARED_LIBS=ON
    # -DCMAKE_CXX_FLAGS=-std=c++11
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}/lib
    -DCMAKE_INCLUDE_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}/include
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    )

# =====================
# Messages and Services
# =====================
add_message_files(DIRECTORY msg)
add_service_files(FILES SetPose.srv)
generate_messages(DEPENDENCIES nav_msgs geometry_msgs std_msgs)

include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIR}
    )

add_executable(${PROJECT_NAME} src/vicon.cpp)
add_dependencies(${PROJECT_NAME} ${VICON_DRIVER})
target_link_libraries(${PROJECT_NAME}
    ${CATKIN_DEVEL_PREFIX}/lib/lib${VICON_DRIVER}${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )

catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES ${PROEJCT_NAME}
    CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_srvs message_runtime
    DEPENDS Eigen Boost
    )

# ============
# Dependencies
# ============
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
