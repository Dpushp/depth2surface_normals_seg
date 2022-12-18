cmake_minimum_required(VERSION 2.8.3)
project(depth2surface_normals_seg)

set (OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	OpenCV REQUIRED
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    image_transport
    dynamic_reconfigure
    )

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Denoise.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_tutorials
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
#  DEPENDS system_lib
)

## Declare a catkin package
catkin_package()


## Build ball_detection
include_directories( include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

file(GLOB UTILITY_FILE
    src/depth_segmentation.cpp
)

add_executable(sim_segmentation_node  
    ${UTILITY_FILE}
    src/depth2surface_normals_seg.cpp)

# make sure configure headers are built before any node using them
add_dependencies(sim_segmentation_node ${PROJECT_NAME}_gencfg)

target_link_libraries(sim_segmentation_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
