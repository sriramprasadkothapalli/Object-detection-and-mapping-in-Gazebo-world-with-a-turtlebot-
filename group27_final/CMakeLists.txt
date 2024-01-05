cmake_minimum_required(VERSION 3.8)
project(group27_final)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include folder whish has all the header files
include_directories(include/group27_final)

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  ros2_aruco_interfaces
  nav_msgs
  nav2_msgs
  mage_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------
add_executable( waffle_broadcaster src/waffle_bot.cpp)
ament_target_dependencies(waffle_broadcaster ${FRAME_DEMO_INCLUDE_DEPENDS})

install(TARGETS waffle_broadcaster
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


add_executable(wafflebot_navigation src/wafflebot_navigation.cpp)
ament_target_dependencies(wafflebot_navigation ${FRAME_DEMO_INCLUDE_DEPENDS})

install(TARGETS wafflebot_navigation
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# add_executable(battery src/battery.cpp src/utils.cpp)
# ament_target_dependencies(battery ${FRAME_DEMO_INCLUDE_DEPENDS})

# install(TARGETS battery
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

#-----------------------------
# Installation
#-----------------------------
install(TARGETS
  waffle_broadcaster
  wafflebot_navigation
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()