cmake_minimum_required(VERSION 2.8.3)
project(qr_driver_alpha)

add_definitions(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  cmake_modules
  pluginlib
  roscpp

  actionlib
  controller_manager
)

# Include a custom cmake file for TinyXML
find_package(TinyXML REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES
#[[    ${PROJECT_NAME}_parser
    ${PROJECT_NAME}_loader
    ${PROJECT_NAME}_loader_plugins]]
  INCLUDE_DIRS
    include
  DEPENDS
    TinyXML
    pluginlib
    roscpp
)

###########
## Build ##
###########

# Build
include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

set(SRC_ROOT        src/middleware)
set(SRC_HW_PATH     ${SRC_ROOT}/hardware)
set(SRC_PROPA_PATH  ${SRC_ROOT}/propagate)
set(SRC_UTIL_PATH   ${SRC_ROOT}/util)

set(INC_ROOT        include/middleware)
set(INC_HW_PATH     ${INC_ROOT}/hardware)
set(INC_PROPA_PATH  ${INC_ROOT}/propagate)
set(INC_UTIL_PATH   ${INC_ROOT}/util)

set(HW_FILES  ${SRC_HW_PATH}/hw_unit.cpp
              ${SRC_HW_PATH}/joint.cpp
)

set(PROPA_FILES   ${SRC_PROPA_PATH}/propagate.cpp
                  ${SRC_PROPA_PATH}/pcan.cpp
                  # ${SRC_PROPA_PATH}/com.cpp
)

set(BIN_FILES src/main.cpp
              ${SRC_ROOT}/middleware.cpp
              ${SRC_ROOT}/ros_robothw.cpp
              ${SRC_ROOT}/ros_wrapper.cpp
              ${SRC_UTIL_PATH}/parser.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}_hw_unit
  ${HW_FILES}
  ${INC_UTIL_PATH}/proto/dragon.pb.cc
)

add_library(${PROJECT_NAME}_propagate
  ${PROPA_FILES}
  ${INC_UTIL_PATH}/proto/dragon.pb.cc
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_exe
  ${BIN_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_hw_unit
  ${catkin_LIBRARIES}
  glog
  protobuf
)

target_link_libraries(${PROJECT_NAME}_propagate
  ${catkin_LIBRARIES}
  glog
  protobuf
#  pcanbasic
)

target_link_libraries(${PROJECT_NAME}_exe
  ${catkin_LIBRARIES}
  glog
#  pcanbasic
  ${PROJECT_NAME}_hw_unit
  ${PROJECT_NAME}_propagate
)

#############
## Install ##
#############

install(DIRECTORY include/middleware/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_hw_unit
                ${PROJECT_NAME}_propagate
                ${PROJECT_NAME}_exe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir images launch config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############
#[[
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(simple_transmission_test           test/simple_transmission_test.cpp)
  catkin_add_gtest(differential_transmission_test     test/differential_transmission_test.cpp)
  catkin_add_gtest(four_bar_linkage_transmission_test test/four_bar_linkage_transmission_test.cpp)

  catkin_add_gtest(transmission_interface_test        test/transmission_interface_test.cpp)
  target_link_libraries(transmission_interface_test ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

  catkin_add_gtest(transmission_parser_test test/transmission_parser_test.cpp)
  target_link_libraries(transmission_parser_test ${PROJECT_NAME}_parser)

  catkin_add_gtest(simple_transmission_loader_test test/simple_transmission_loader_test.cpp)
  target_link_libraries(simple_transmission_loader_test ${PROJECT_NAME}_parser)

  catkin_add_gtest(differential_transmission_loader_test test/differential_transmission_loader_test.cpp)
  target_link_libraries(differential_transmission_loader_test ${PROJECT_NAME}_parser)

  catkin_add_gtest(four_bar_linkage_transmission_loader_test test/four_bar_linkage_transmission_loader_test.cpp)
  target_link_libraries(four_bar_linkage_transmission_loader_test ${PROJECT_NAME}_parser)

  catkin_add_gtest(transmission_interface_loader_test test/transmission_interface_loader_test.cpp)
  target_link_libraries(transmission_interface_loader_test ${PROJECT_NAME}_parser
                                                          transmission_interface_loader)
endif()]]
