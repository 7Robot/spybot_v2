cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)


find_library(PIGPIO_LIBRARY pigpio /usr/local/lib)

catkin_package(
	DEPENDS pigpio
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
	${PIGPIO_LIBRARY}
)
