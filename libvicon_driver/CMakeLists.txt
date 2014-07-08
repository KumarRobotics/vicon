cmake_minimum_required(VERSION 2.6.2)
project(vicon_driver CXX)

find_package(Boost 1.40 REQUIRED)

if(WIN32)
  message(FATAL_ERROR "Windows is not yet supported")
endif(WIN32)

if(NOT APPLE)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SDK_DIR "${PROJECT_SOURCE_DIR}/vicon_sdk/Linux64")
    find_library(VICON_BOOST_THREAD_LIB libboost_thread-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SYSTEM_LIB libboost_system-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_CHRONO_LIB libboost_chrono-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    set(VICON_BOOST_LIBS ${VICON_BOOST_THREAD_LIB} ${VICON_BOOST_SYSTEM_LIB} ${VICON_BOOST_CHRONO_LIB})
  else(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SDK_DIR "${PROJECT_SOURCE_DIR}/vicon_sdk/Linux32")
    find_library(VICON_BOOST_WAVE_LIB libboost_wave-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_THREAD_LIB libboost_thread-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SYSTEM_LIB libboost_system-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SERIALIZATION_LIB libboost_serialization-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_DATE_TIME_LIB libboost_date_time-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_CHRONO_LIB libboost_chrono-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_FILESYSTEM_LIB libboost_filesystem-mt.so.1.51.0 ${SDK_DIR} NO_DEFAULT_PATH)
    set(VICON_BOOST_LIBS ${VICON_BOOST_WAVE_LIB} ${VICON_BOOST_THREAD_LIB} ${VICON_BOOST_SYSTEM_LIB}
      ${VICON_BOOST_SERIALIZATION_LIB} ${VICON_BOOST_DATE_TIME_LIB} ${VICON_BOOST_CHRONO_LIB} ${VICON_BOOST_FILESYSTEM_LIB})
  endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
  #message(STATUS "SDK_DIR: ${SDK_DIR}")
  #message(STATUS "VICON_BOOST_LIBS: ${VICON_BOOST_LIBS}")
else(NOT APPLE)
  set(SDK_DIR "${PROJECT_SOURCE_DIR}/vicon_sdk/MacOSX")
endif(NOT APPLE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
# Support new yaml-cpp API (yaml-cpp>=0.5)
if(${YAMLCPP_FOUND})
  if(NOT(${YAMLCPP_VERSION} VERSION_LESS 0.5))
    add_definitions(-DHAVE_NEW_YAMLCPP)
  endif(NOT(${YAMLCPP_VERSION} VERSION_LESS 0.5))
endif(${YAMLCPP_FOUND})

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${SDK_DIR}")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Force "no-as-needed", required for ViconSDK lib
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")

add_library(vicon_driver SHARED src/vicon_driver.cpp src/vicon_calib.cpp)

include_directories("${PROJECT_SOURCE_DIR}/include" ${SDK_DIR} ${EIGEN3_INCLUDE_DIR})
find_library(VICON_SDK_LIB ViconDataStreamSDK_CPP ${SDK_DIR} NO_DEFAULT_PATH)
target_link_libraries(vicon_driver ${VICON_SDK_LIB} ${VICON_BOOST_LIBS} ${YAMLCPP_LIBRARIES})

#set_target_properties(vicon_driver PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS vicon_driver LIBRARY DESTINATION lib)
