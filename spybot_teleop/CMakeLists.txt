cmake_minimum_required(VERSION 2.8.3)
project(spybot_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spybot_teleop
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_key src/teleop.cpp)

target_link_libraries(teleop_key
	${catkin_LIBRARIES}
)
