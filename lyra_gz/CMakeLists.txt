cmake_minimum_required(VERSION 3.5)
project(lyra_gz)

find_package(ament_cmake REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#TODO Do we actually really need these?
find_package(lyra_msg REQUIRED)
find_package(rclcpp REQUIRED)
#TODO what about this?
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

#TODO: Permasticks the models in the Gazebo directory.
#	It would be better if the models went away when the package did,
#	in case they are somehow causing problems and a clean removal is necessary.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY world DESTINATION share/${PROJECT_NAME})
set(GAZEBO_MODEL_PATH $ENV{HOME}/.gazebo)
install(DIRECTORY models
	DESTINATION ${GAZEBO_MODEL_PATH}
)

ament_package()
