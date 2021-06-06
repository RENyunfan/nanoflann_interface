cmake_minimum_required(VERSION 2.8.3)
project(nanoflann_interface)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        pcl_ros
        geometry_msgs
        nav_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES nanoflann_interface
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
#add_library(nanoflann_interface
#        src/nanoflann_interface.cpp
#        )
#target_link_libraries(nanoflann_interface
#        ${THIRD_PARTY}
#        )
#
add_executable(nanotree_test
        Apps/unit_test/kd_tree_test.cpp
        src/nanoflann_interface.cpp
        )
target_link_libraries(nanotree_test

        ${THIRD_PARTY}
        )
