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

set(SRC_ROOT            src)

set(SRC_SYS             ${SRC_ROOT}/system)
set(SRC_SYS_FOUNDATION  ${SRC_SYS}/foundation)
set(SRC_SYS_PLATFORM    ${SRC_SYS}/platform)
set(SRC_SYS_ROBOT       ${SRC_SYS}/robot)
set(SRC_SYS_UTILS       ${SRC_SYS}/utils)

set(SRC_REP             ${SRC_ROOT}/repository)
set(SRC_REP_RES         ${SRC_REP}/resource)
set(SRC_REP_EST         ${SRC_REP}/estimator)

set(SRC_APP             ${SRC_ROOT}/apps)


set(SYS_FLATFORM_FILES  ${SRC_SYS_FOUNDATION}/label.cpp
                        ${SRC_SYS_FOUNDATION}/auto_instanceor.cpp
                        ${SRC_SYS_FOUNDATION}/cfg_reader.cpp

                        ${SRC_REP_RES}/joint.cpp
                        ${SRC_REP_RES}/force_sensor.cpp
                        ${SRC_REP_RES}/joint_manager.cpp

                        ${SRC_REP_EST}/touchdown_estimator.cpp
                        
                        ${SRC_SYS_PLATFORM}/sw_node/sw_node_manager.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/sw_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/leg_node.cpp
                        ${SRC_SYS_PLATFORM}/propagate/propagate_manager.cpp
                        ${SRC_SYS_PLATFORM}/propagate/propagate.cpp
                        ${SRC_SYS_PLATFORM}/propagate/pcan_fake.cpp
                        ${SRC_SYS_PLATFORM}/propagate/pcan.cpp
                        
                        ${SRC_SYS_PLATFORM}/thread/threadpool.cpp

                        ${SRC_SYS_PLATFORM}/master.cpp
)

set(SYS_ROBOT_FILES     ${SRC_SYS_ROBOT}/mii_robot.cpp
)

set(BIN_FILES           ${SRC_ROOT}/main.cpp
                        ${SRC_APP}/ros_robothw.cpp
                        ${SRC_APP}/ros_wrapper.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}_sys_platform
  ${SYS_FLATFORM_FILES}
)

add_library(${PROJECT_NAME}_sys_robot
  ${SYS_ROBOT_FILES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}
  ${BIN_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_sys_platform
  ${catkin_LIBRARIES}
  glog
  pcanbasic
)

target_link_libraries(${PROJECT_NAME}_sys_robot
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sys_platform
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sys_robot
)

#############
## Install ##
#############

install(DIRECTORY include/middleware/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_sys_platform
                ${PROJECT_NAME}_sys_robot
                ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir images launch config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
