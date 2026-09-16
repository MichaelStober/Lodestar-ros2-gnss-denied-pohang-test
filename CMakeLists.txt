cmake_minimum_required(VERSION 3.16)
project(lodestar_odometry)

# ---------------------------------------------------------------- compiler ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-sign-compare)

# ------------------------------------------------------------- ROS 2 deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)

# ------------------------------------------------------- system libraries ---
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters kdtree search)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres 2.0 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenMP REQUIRED)

add_definitions(${PCL_DEFINITIONS})

set(ROS_DEPS
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  cv_bridge
  pcl_conversions
  angles
)

# ---------------------------------------------------------------- library ---
set(${PROJECT_NAME}_LIB_SRCS
  src/${PROJECT_NAME}/utils.cpp
  src/${PROJECT_NAME}/pointnormal.cpp
  src/${PROJECT_NAME}/registration.cpp
  src/${PROJECT_NAME}/n_scan_normal.cpp
  src/${PROJECT_NAME}/odometrykeyframefuser.cpp
  src/${PROJECT_NAME}/eval_trajectory.cpp
  src/${PROJECT_NAME}/statistics.cpp
  src/${PROJECT_NAME}/lodestar.cpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_LIB_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
target_link_directories(${PROJECT_NAME} PUBLIC ${PCL_LIBRARY_DIRS})

ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPS})

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

# ------------------------------------------------------------- executable ---
add_executable(lodestar_odom src/lodestar_odom.cpp)
ament_target_dependencies(lodestar_odom ${ROS_DEPS})
target_link_libraries(lodestar_odom
  ${PROJECT_NAME}
  Boost::program_options
)

# ----------------------------------------------------------------- install ---
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lodestar_odom
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/run_lodestar_odom.sh
    scripts/check_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS_DEPS} PCL OpenCV Ceres Eigen3)

ament_package()
