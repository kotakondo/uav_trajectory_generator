cmake_minimum_required(VERSION 3.14.4)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(trajectory_generator)

add_compile_options("-O3")

# check c++14 / c++0x
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
# if(COMPILER_SUPPORTS_CXX14)
#     set(CMAKE_CXX_FLAGS "-std=c++14")
# else()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support.")
# endif()

# check c++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

# ROS1 stuff
# if (NOT CMAKE_BUILD_TYPE)
#     # Options: Debug, Release, MinSizeRel, RelWithDebInfo
#     message(STATUS "No build type selected, default to Release")
#     set(CMAKE_BUILD_TYPE "Release")
# endif()
 
# Replace above with:
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(snapstack_msgs REQUIRED)

## Find system libraries
find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###This catkin_package call needs to be replaced with various ament_cmake calls
# ROS1:
# catkin_package(
#   INCLUDE_DIRS
#     include
#     ${EIGEN3_INCLUDE_DIR}
# #  LIBRARIES
# #    ${PROJECT_NAME}_core
#   CATKIN_DEPENDS
#     roscpp
#     snapstack_msgs
#     geometry_msgs
# #  DEPENDS
# #    Boost
# )

###########
## Build ##
###########

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
)

################## Replacing catkin_package() ####################

# ROS2:
# Replaces INCLUDE_DIRS keyword of catkin_package() call
target_include_directories(${PROJECT_NAME}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIR} 
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  src/TrajectoryGenerator.cpp
  src/trajectories/Circle.cpp
  src/trajectories/Line.cpp
)

# ROS2: Shouldn't have to do this as ament already handles inter-package dependencies.
# add_dependencies(${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
# )

## Specify libraries to link executable targets against
#target_link_libraries(${PROJECT_NAME}_core
#  ${catkin_LIBRARIES}
#)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${ament_TARGETS}
  rclcpp::rclcpp
  snapstack_msgs::snapstack_msgs 
  ${geometry_msgs_TARGETS} 
  ${std_msgs_TARGETS}
)

#############
## Install ##
#############

# ROS2:
# Replaces LIBRARIES keyword from catkin_package()
# Mark executables and/or libraries for installation 
install(TARGETS ${PROJECT_NAME} EXPORT export_package
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

# Mark header files for installation 
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${project_name}
  FILES_MATCHING PATTERN "*.hpp"
)

# Export for downstream users
ament_export_targets(export_package) 

# ROS2: 
# Replaces CATKIN_DEPENDS keyword from catkin_package()
ament_export_dependencies(
  rclcpp 
  snapstack_msgs 
  geometry_msgs 
  std_msgs 
)

ament_package()