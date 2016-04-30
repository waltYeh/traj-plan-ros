cmake_minimum_required(VERSION 2.8.3)
project(r2d2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
#  geometry_msgs
  message_generation
  genmsg
)

add_message_files(
  FILES
  states.msg
  commands.msg
  output.msg
  control_sp.msg
)

generate_messages(
   DEPENDENCIES
  std_msgs
#  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

set(wiringPi_include "/usr/local/include")

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)
add_library(commons SHARED src/commons.cpp)
add_library(nurbs SHARED src/nurbs.cpp)
add_library(smooth SHARED src/tangential_smooth.cpp)
add_executable(linker src/linker.cpp)
target_link_libraries(linker
  commons 
 ${catkin_LIBRARIES} 
  wiringPi)
add_dependencies(linker r2d2_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener 
  commons
  ${catkin_LIBRARIES})
add_dependencies(listener r2d2_cpp)

add_executable(commander src/commander.cpp)
target_link_libraries(commander 
  commons
  nurbs
  smooth
  ${catkin_LIBRARIES})
add_dependencies(commander r2d2_cpp)

add_executable(controller src/controller.cpp)
target_link_libraries(controller 
  commons
  ${catkin_LIBRARIES})
add_dependencies(controller r2d2_cpp)
