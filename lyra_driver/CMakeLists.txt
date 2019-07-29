cmake_minimum_required(VERSION 3.5)
project(lyra_driver)

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


add_executable(lyra_bridge src/lyra.cpp)
ament_target_dependencies(lyra_bridge "rclcpp" "lyra_msg" "std_msgs" "builtin_interfaces")
#rosidl_target_interfaces(lyra_bridge lyra_bridge "rosidl_typesupport_cpp")#Necessary if you are including messages in the same package they are defined in: https://discourse.ros.org/t/ros2-how-to-use-custom-message-in-project-where-its-declared/2071

#TODO What is the significance of this??
install(TARGETS
	lyra_bridge
#  publisher_member_function
#  publisher_not_composable
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
