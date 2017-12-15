cmake_minimum_required(VERSION 2.8.3)
project(qr_driver)

add_definitions(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  class_loader
)

# Include a custom cmake file for TinyXML
find_package(TinyXML REQUIRED)
find_package(Eigen3  REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  DEPENDS
    TinyXML
    class_loader
)

###########
## Build ##
###########

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_ROOT            src)
set(SRC_TEST_ROOT       test)
set(SRC_SYS_FOUNDATION  ${SRC_ROOT}/foundation)

set(SRC_SYS             ${SRC_ROOT}/system)
set(SRC_SYS_PLATFORM    ${SRC_SYS}/platform)
set(SRC_SYS_ROBOT       ${SRC_SYS}/robot)
set(SRC_SYS_UTILS       ${SRC_SYS}/utils)

set(SRC_REP             ${SRC_ROOT}/repository)
set(SRC_REP_RES         ${SRC_REP}/resource)
set(SRC_REP_EST         ${SRC_REP}/estimator)
set(SRC_REP_BOX         ${SRC_REP}/control_toolbox)

# Files define.
set(SYS_FOUDATION_FILES ${SRC_SYS_FOUNDATION}/label.cpp
                        ${SRC_SYS_FOUNDATION}/auto_instanceor.cpp

                        ${SRC_SYS_FOUNDATION}/cfg_reader.cpp
                        ${SRC_SYS_FOUNDATION}/internal/mii_config.cpp
                        ${SRC_SYS_FOUNDATION}/internal/expr.cpp
                        
)

set(SYS_REGISTRY_FILES  ${SRC_REP}/registry.cpp
)

set(SYS_FLATFORM_FILES  ${SRC_REP_RES}/imu_sensor.cpp
                        ${SRC_REP_RES}/joint.cpp
                        ${SRC_REP_RES}/motor.cpp
                        ${SRC_REP_RES}/power.cpp
                        ${SRC_REP_RES}/force_sensor.cpp
                        ${SRC_REP_RES}/joint_manager.cpp

                        ${SRC_REP_EST}/touchdown_estimator.cpp

                        ${SRC_REP_BOX}/pid.cpp
                        
                        ${SRC_SYS_PLATFORM}/sw_node/sw_node_manager.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/sw_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/leg_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/imu_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/power_node.cpp

                        ${SRC_SYS_PLATFORM}/propagate/propagate_manager.cpp
                        ${SRC_SYS_PLATFORM}/propagate/propagate.cpp
                        ${SRC_SYS_PLATFORM}/propagate/pcan_fake.cpp
                        ${SRC_SYS_PLATFORM}/propagate/pcan.cpp
                        ${SRC_SYS_PLATFORM}/propagate/arm_pcan.cpp
                        ${SRC_SYS_PLATFORM}/propagate/motor_pcan.cpp
                        ${SRC_SYS_PLATFORM}/propagate/usb.cpp
                        ${SRC_SYS_PLATFORM}/propagate/imu_usb.cpp
                        
                        ${SRC_SYS_PLATFORM}/thread/threadpool.cpp

                        ${SRC_SYS_PLATFORM}/master.cpp
)

set(SYS_ROBOT_FILES     ${SRC_SYS_ROBOT}/mii_robot.cpp
)

set(SYS_TEST_FILES      ${SRC_TEST_ROOT}/main.cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}_foundation
  ${SYS_FOUDATION_FILES}
)

add_library(${PROJECT_NAME}_registry
  ${SYS_REGISTRY_FILES}
)

add_library(${PROJECT_NAME}_sys_platform
  ${SYS_FLATFORM_FILES}
)

add_library(${PROJECT_NAME}_sys_robot
  ${SYS_ROBOT_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_foundation
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  glog
)

target_link_libraries(${PROJECT_NAME}_registry
  ${EIGEN3_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_sys_platform
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PROJECT_NAME}_foundation
  pcanbasic
)

target_link_libraries(${PROJECT_NAME}_sys_robot
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sys_platform
  ${PROJECT_NAME}_registry
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_foundation
                ${PROJECT_NAME}_registry
                ${PROJECT_NAME}_sys_platform
                ${PROJECT_NAME}_sys_robot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir include images config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}_test
  ${SYS_TEST_FILES}
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}_sys_robot
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)