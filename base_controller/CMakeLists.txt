cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)


find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2 /usr/local/lib)

catkin_package(
	DEPENDS pigpiod_if2
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

add_executable(controller src/controller.cpp)

target_link_libraries(controller
	${catkin_LIBRARIES}
	${PIGPIOD_IF2_LIBRARY}
)
