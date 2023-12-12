cmake_minimum_required(VERSION 3.0.2)
project(rm_drive_analyzer)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        rosconsole
        rm_ecat_slave
        soem_interface
        message_logger

        diagnostic_aggregator
        diagnostic_msgs
        pluginlib
        )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        rosconsole
        rm_ecat_slave
        soem_interface
        message_logger

        diagnostic_aggregator
        diagnostic_msgs
        pluginlib
        LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(rm_ecat_analyzer
        src/RmEcatAnalyzer.cpp)

add_executable(node src/node.cpp src/RmEcatDiagnosticPublisher.cpp src/RmEcatAggregator.cpp)

## 用于链接 rosconsole 库的变量。
# 动态库需要，将自己的库与对应的库链接起来
target_link_libraries(node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(
#        TARGETS ${PROJECT_NAME}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark cpp header files for installation
#install(
#        DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h"
#)
#
## Mark other files for installation
#install(
#        FILES rm_calibration_controllers_plugins.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()