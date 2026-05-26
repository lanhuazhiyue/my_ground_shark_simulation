cmake_minimum_required(VERSION 2.8.3)

project(ground_shark_mid360)

find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
	roslaunch
	roscpp
	sensor_msgs
	livox_ros_driver2
	pcl_conversions
	pcl_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

foreach(dir config launch meshes urdf) 
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(custom2pc2 src/custom2pc2.cpp)
target_link_libraries(custom2pc2 ${catkin_LIBRARIES})
