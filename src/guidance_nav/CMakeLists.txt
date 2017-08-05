cmake_minimum_required(VERSION 2.8.3)
project(guidance_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  actionlib_msgs 
  actionlib
)

find_package(OpenCV REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# add_message_files(
#   FILES
#   Message1.msg
# )

# add_service_files(
#  FILES
#)

add_action_files(
  FILES
  GuidanceNav.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
 	${catkin_INCLUDE_DIRS}
)

add_executable(guidance_nav src/guidance_nav_node.cpp
  src/guidance_nav.cpp
  src/pid.cpp 
)

add_dependencies(guidance_nav
  guidance_nav_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(guidance_nav
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_dependencies(guidance_nav guidance_nav_generate_messages_cpp)

