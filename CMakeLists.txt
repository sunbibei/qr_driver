cmake_minimum_required(VERSION 2.8.3)
project(qr_driver)

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

# Transmission parser Library
#[[add_library(${PROJECT_NAME}_parser
  src/transmission_parser.cpp include/transmission_interface/transmission_parser.h
)
target_link_libraries(${PROJECT_NAME}_parser ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})


# Transmission loader library
add_library(${PROJECT_NAME}_loader
  src/transmission_loader.cpp           include/transmission_interface/transmission_loader.h
  src/transmission_interface_loader.cpp include/transmission_interface/transmission_interface_loader.h
)
target_link_libraries(${PROJECT_NAME}_loader
  ${PROJECT_NAME}_parser
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES})


add_library(${PROJECT_NAME}_loader_plugins
  src/simple_transmission_loader.cpp           include/transmission_interface/simple_transmission_loader.h
  src/differential_transmission_loader.cpp     include/transmission_interface/differential_transmission_loader.h
  src/four_bar_linkage_transmission_loader.cpp include/transmission_interface/four_bar_linkage_transmission_loader.h
  src/joint_state_interface_provider.cpp       include/transmission_interface/joint_state_interface_provider.h
  src/position_joint_interface_provider.cpp    include/transmission_interface/position_joint_interface_provider.h
  src/velocity_joint_interface_provider.cpp    include/transmission_interface/velocity_joint_interface_provider.h
  src/effort_joint_interface_provider.cpp      include/transmission_interface/effort_joint_interface_provider.h)
target_link_libraries(${PROJECT_NAME}_loader_plugins ${PROJECT_NAME}_loader)]]


set(HW_PATH include/middleware/hardware)
set(PROPA_PATH include/middleware/propagate)

set(HW_FILES  ${HW_PATH}/hw_unit.cpp
              ${HW_PATH}/joint.cpp
              ${HW_PATH}/encoder.cpp
              ${HW_PATH}/motor.cpp
)

set(PROPA_FILES   ${PROPA_PATH}/propagate.cpp
                  ${PROPA_PATH}/pcan.cpp
                  # ${PROPA_PATH}/com.cpp
)

set(BIN_FILES src/main.cpp
              include/middleware/middleware.cpp
              include/middleware/ros_robothw.cpp
              include/middleware/ros_wrapper.cpp
              include/middleware/util/parser.cpp
)

## Declare a C++ library
add_library(hardware_lib
  ${HW_FILES}
)

add_library(propagate_lib
  ${PROPA_FILES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_exe
  ${BIN_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(hardware_lib
  ${catkin_LIBRARIES}
  glog
)

target_link_libraries(propagate_lib
  ${catkin_LIBRARIES}
  glog
#  pcanbasic
)

target_link_libraries(${PROJECT_NAME}_exe
  ${catkin_LIBRARIES}
  glog
#  pcanbasic
  hardware_lib
  propagate_lib
)

#############
## Install ##
#############

install(DIRECTORY include/middleware/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS hardware_lib
                propagate_lib
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
