cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs serial geometry_msgs nav_msgs sensor_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  serial
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}${EIGEN3_INCLUDE_DIR})
add_definitions( ${EIGEN_DEFINITIONS})



catkin_package(
  CATKIN_DEPENDS
  message_runtime
          nav_msgs
        roscpp
        sensor_msgs
        std_msgs
)


include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_executable(base_controller
  src/base_controller.cpp
)
add_dependencies(base_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_controller
  ${catkin_LIBRARIES}
)

add_library(localEKF
  include/base_controller/localEKF.h
  src/localEKF.cpp
)
add_dependencies(localEKF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localEKF
  ${catkin_LIBRARIES}
)

add_library(Sensorfusion
  include/base_controller/Sensorfusion.h
  src/Sensorfusion.cpp
)
add_dependencies(Sensorfusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Sensorfusion localEKF
  ${catkin_LIBRARIES}
)


add_executable(Sensorfusion_node
  src/Sensorfusion_node.cpp
)
add_dependencies(Sensorfusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Sensorfusion_node Sensorfusion
  ${catkin_LIBRARIES}
)

add_executable(imu_pre_inte
  src/imu_pre_inte.cpp
)
add_dependencies(imu_pre_inte ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_pre_inte
  ${catkin_LIBRARIES}
)