cmake_minimum_required(VERSION 3.0.2)
project(unicycle_control)

add_compile_options(-std=c++17)
add_compile_options(-ffast-math)
add_compile_options(-Wno-pmf-conversions)

set (Torch_DIR "/opt/libtorch/share/cmake/Torch")

find_package(catkin REQUIRED COMPONENTS
  nlib
  roscpp
  rospy
  std_msgs
)

find_package (Torch REQUIRED)

catkin_package (
	INCLUDE_DIRS include ${TORCH_INCLUDE_DIRS}
	CATKIN_DEPENDS nlib
)

include_directories (
	include
	${TORCH_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable (unicycle_control
	src/unicycle_control_node.cpp
	src/unicycle_control_modflow.cpp
)
target_link_libraries (unicycle_control dl ${catkin_LIBRARIES} ${TORCH_LIBRARIES})
