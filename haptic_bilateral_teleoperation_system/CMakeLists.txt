cmake_minimum_required(VERSION 3.8)
project(haptic_bilateral_teleoperation_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)          
find_package(tf2 REQUIRED)              
find_package(tf2_geometry_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(franka_msgs REQUIRED) 


#for simulation:
add_executable(hbts_bridge_sim src/hbts_bridge_sim.cpp)
ament_target_dependencies(hbts_bridge_sim 
  rclcpp 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  tf2
  omni_msgs
  visualization_msgs
)
target_include_directories(hbts_bridge_sim PRIVATE
   include
)


add_executable(hbts_bridge_franka src/hbts_bridge_franka.cpp)
ament_target_dependencies(hbts_bridge_franka 
  rclcpp 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  tf2
  omni_msgs
  franka_msgs
  visualization_msgs
)
target_include_directories(hbts_bridge_franka PRIVATE
   include
)



add_executable(sensor_calibration_sim src/sensor_calibration_sim.cpp)
ament_target_dependencies(sensor_calibration_sim
  rclcpp
  geometry_msgs
  franka_msgs
)
target_include_directories(sensor_calibration_sim PRIVATE
   include
)


add_executable(sensor_calibration_franka src/sensor_calibration_franka.cpp)
ament_target_dependencies(sensor_calibration_franka
  rclcpp
  geometry_msgs
  franka_msgs
)
target_include_directories(sensor_calibration_franka PRIVATE
   include
)


add_executable(end_effector_pose src/end_effector_pose.cpp)
ament_target_dependencies(end_effector_pose
  rclcpp
  geometry_msgs
  franka_msgs
)



install(TARGETS
  hbts_bridge_sim
  hbts_bridge_franka
  sensor_calibration_sim
  sensor_calibration_franka
  end_effector_pose
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
