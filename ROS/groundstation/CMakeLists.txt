cmake_minimum_required(VERSION 2.8.3)
project(robot_groundstation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
  tf
  genmsg
  std_msgs
  message_generation
  sepanta_msgs
  protobuf_msgs
  config_server
  infrastructure
  cv_bridge
  image_transport
  )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ")
set(CMAKE_CXX_FLAGS_RELEASE "-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DROBOLAND_DEBUG")

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(GOBJECT REQUIRED gobject-2.0)

include_directories(
  ${GLIB_INCLUDE_DIRS}
)

link_directories(
  ${GLIB_LIBRARY_DIRS}
)

set(GSTREAMER_MINIMUM_VERSION 1.0.5)

pkg_check_modules(GST1_TEST gstreamer-1.0)
if ( GST1_TEST_FOUND AND NOT ${GST1_TEST_VERSION} VERSION_LESS ${GSTREAMER_MINIMUM_VERSION} )
    pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
    pkg_check_modules(GSTREAMER-APP REQUIRED gstreamer-app-1.0)
    pkg_check_modules(GSTREAMER-AUDIO REQUIRED gstreamer-audio-1.0)
    pkg_check_modules(GSTREAMER-PBUTILS REQUIRED gstreamer-pbutils-1.0)
    pkg_check_modules(GSTREAMER-FFT REQUIRED gstreamer-fft-1.0)
    add_definitions(-DGST_API_VERSION_1=1)
else()
    # fallback to gstreamer-0.10
    unset(GSTREAMER_MINIMUM_VERSION)
    pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
    pkg_check_modules(GSTREAMER-APP REQUIRED gstreamer-app-0.10)
    pkg_check_modules(GSTREAMER-AUDIO REQUIRED gstreamer-audio-0.10)
    pkg_check_modules(GSTREAMER-FFT REQUIRED gstreamer-fft-0.10)
    set_source_files_properties(WebKitWebAudioSourceGStreamer.cpp PROPERTIES COMPILE_DEFINITIONS "GLIB_DISABLE_DEPRECATION_WARNINGS=1")
endif()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${GOBJECT_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER-APP_INCLUDE_DIRS}
  ${GSTREAMER-AUDIO_INCLUDE_DIRS}
  ${GSTREAMER-PBUTILS_INCLUDE_DIRS}
  ${GSTREAMER-FFT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${GOBJECT_LIBRARY_DIRS}
  ${GLIB_LIBRARY_DIRS}
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTREAMER-APP_LIBRARY_DIRS}
  ${GSTREAMER-AUDIO_LIBRARY_DIRS}
  ${GSTREAMER-PBUTILS_LIBRARY_DIRS}
  ${GSTREAMER-FFT_LIBRARY_DIRS}
)

set(inputtest_LIBRARIES

  ${OpenCV_LIBS}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER-APP_LIBRARIES}
  ${GSTREAMER-AUDIO_LIBRARIES}
  ${GSTREAMER-PBUTILS_LIBRARIES}
  ${GSTREAMER-FFT_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${GOBJECT_LIBRARIES}

  pthread
  ${GLIB_LIBRARIES}
  ${GLIB_GIO_LIBRARIES}
  ${GLIB_GOBJECT_LIBRARIES}
)


catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS message_runtime 
      LIBRARIES robotgroundstation
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(ctcp src/tcp/tcpacceptor.cpp src/tcp/tcpconnector.cpp src/tcp/tcpstream.cpp )

#Add executables
add_executable(groundstation_node src/groundstation_node.cpp src/groundstation_ros.cpp )
target_link_libraries(groundstation_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} robogroundstation)
add_dependencies(groundstation_node sepanta_msgs_generate_messages_cpp config_server_generate_messages_cpp)

add_library(robogroundstation src/groundstation.cpp  src/network.cpp)
target_link_libraries(robogroundstation ${catkin_LIBRARIES} ${inputtest_LIBRARIES} zmq )
 
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)
