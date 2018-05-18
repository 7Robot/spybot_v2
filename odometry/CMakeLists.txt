cmake_minimum_required(VERSION 2.8.3)
project(odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)


find_library(pigpio_LIBRARIES pigpio /usr/local/lib)

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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/odometry.cpp
# )


add_executable(odometry src/odometry.cpp src/rotary_encoder.cpp)

target_link_libraries(odometry 
	${catkin_LIBRARIES} 
	${pigpio_LIBRARIES}
)
