cmake_minimum_required(VERSION 3.5)
project(lyra_driver)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	leap_motion
)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

#TODO Do we actually really need these?
#find_package(lyra_msg REQUIRED)
#find_package(roscpp REQUIRED)
#TODO what about this?
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(leap_motion REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
	DEPENDS roscpp
	CATKIN_DEPENDS roscpp leap_motion
)


add_executable(lyra_bridge src/lyra.cpp)
target_link_libraries(lyra_bridge ${catkin_LIBRARIES})

add_executable(lyra_polaris src/aion_polaris.cpp)
target_link_libraries(lyra_polaris ${catkin_LIBRARIES})

add_executable(lyra_leap src/aion_leap.cpp)
target_link_libraries(lyra_leap ${catkin_LIBRARIES})
