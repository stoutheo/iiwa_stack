cmake_minimum_required(VERSION 3.5)
project(iiwa_ros_py)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  cmake_modules
)

catkin_package(
  #	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS iiwa_msgs
)

## Add dependence to the iiwa_msg module for the library
#add_dependencies(${PROJECT_NAME} iiwa_msgs_generate_messages_cpp)
