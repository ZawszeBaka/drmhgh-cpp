cmake_minimum_required(VERSION 2.8.3)
project(team200)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV 3.4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES api_lane_detection
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(api_lane_detection
  src/lanedetector.cpp
  src/carcontroller.cpp
  src/signrecognizer.cpp
  src/helperfunctions.cpp
  src/svmprocess.cpp
  src/tracker.cpp
)

add_executable(team200 src/main.cpp)

add_dependencies(team200 ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  team200
  api_lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

#install(TARGETS api_lane_detection ${ADDITIONAL_EXECUTABLES}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

message("Open CV version is ${OpenCV_VERSION}")
