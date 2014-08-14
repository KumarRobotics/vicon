cmake_minimum_required(VERSION 2.6.2)
project(vicon_driver CXX)

find_package(Boost 1.40 REQUIRED)

if(WIN32)
  message(FATAL_ERROR "Windows is not yet supported")
endif()

if(NOT APPLE)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SDK_DIR "${PROJECT_SOURCE_DIR}/vicon_sdk/Linux64")
    find_library(VICON_BOOST_THREAD_LIB boost_thread-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SYSTEM_LIB boost_system-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_CHRONO_LIB boost_chrono-mt ${SDK_DIR} NO_DEFAULT_PATH)
    set(VICON_BOOST_LIBS ${VICON_BOOST_THREAD_LIB} ${VICON_BOOST_SYSTEM_LIB} ${VICON_BOOST_CHRONO_LIB})
  else(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SDK_DIR "${PROJECT_SOURCE_DIR}/vicon_sdk/Linux32")
    find_library(VICON_BOOST_WAVE_LIB boost_wave-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_THREAD_LIB boost_thread-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SYSTEM_LIB boost_system-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SERIALIZATION_LIB boost_serialization-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_DATE_TIME_LIB boost_date_time-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_CHRONO_LIB boost_chrono-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_FILESYSTEM_LIB boost_filesystem-mt ${SDK_DIR} NO_DEFAULT_PATH)
    set(VICON_BOOST_LIBS ${VICON_BOOST_WAVE_LIB} ${VICON_BOOST_THREAD_LIB} ${VICON_BOOST_SYSTEM_LIB}
      ${VICON_BOOST_SERIALIZATION_LIB} ${VICON_BOOST_DATE_TIME_LIB} ${VICON_BOOST_CHRONO_LIB} ${VICON_BOOST_FILESYSTEM_LIB})
  endif()
  #message(STATUS "SDK_DIR: ${SDK_DIR}")
  #message(STATUS "VICON_BOOST_LIBS: ${VICON_BOOST_LIBS}")
else(NOT APPLE)
  set(SDK_DIR "${PROJECT_SOURCE_DIR}/vicon_sdk/MacOSX")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
# Support new yaml-cpp API (yaml-cpp>=0.5)
if(${YAMLCPP_FOUND})
  if(NOT(${YAMLCPP_VERSION} VERSION_LESS 0.5))
    add_definitions(-DHAVE_NEW_YAMLCPP)
  endif()
endif()

# Force "no-as-needed", required for ViconSDK lib
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-O1 -Wl,--no-as-needed")

set(CMAKE_INSTALL_RPATH ${SDK_DIR})

add_library(vicon_driver SHARED src/vicon_driver.cpp src/vicon_calib.cpp)

include_directories("${PROJECT_SOURCE_DIR}/include" ${SDK_DIR} ${EIGEN3_INCLUDE_DIR})
find_library(VICON_SDK_LIB ViconDataStreamSDK_CPP ${SDK_DIR} NO_DEFAULT_PATH)
find_library(VICON_DEBUG_SERVICES_LIB DebugServices ${SDK_DIR} NO_DEFAULT_PATH)
target_link_libraries(vicon_driver ${VICON_SDK_LIB} ${VICON_DEBUG_SERVICES_LIB} ${VICON_BOOST_LIBS} ${YAMLCPP_LIBRARIES})

install(TARGETS vicon_driver LIBRARY DESTINATION lib)
