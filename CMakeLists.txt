cmake_minimum_required(VERSION 2.8.3)
project(merge_lasers)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	sensor_msgs
	roscpp
	)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
	
add_executable(
	merge_lasers_node 
	src/merge_lasers_node.cpp
	src/merge_lasers.cpp)

target_link_libraries(merge_lasers_node
	${catkin_LIBRARIES}
	)


