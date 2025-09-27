cmake_minimum_required(VERSION 2.8.3)

project(my_sr_arm_rb)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
# Tìm các package cần thiết
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_eigen
  trajectory_msgs
  eigen_conversions
)

# Tìm package Eigen
find_package(Eigen3 REQUIRED)

# Thêm Eigen include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Khai báo file thực thi
add_executable(virtual_constraint_controller_node src/virtual_constraint_controller.cpp)

# Liên kết các thư viện cần thiết
target_link_libraries(virtual_constraint_controller_node
  ${catkin_LIBRARIES}
)