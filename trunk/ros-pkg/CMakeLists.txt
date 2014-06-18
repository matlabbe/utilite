cmake_minimum_required(VERSION 2.8.3)
project(utilite)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
             cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs 
             image_transport tf tf_conversions dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(UtiLite 0.3 REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(Fmodex REQUIRED)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${Fmodex_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   AudioFrame.msg
   AudioFrameFreq.msg
   AudioFrameFreqSqrdMagn.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/AudioSave.cfg cfg/Camera.cfg cfg/MotionFilter.cfg cfg/rgb2ind.cfg cfg/xy2polar.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs 
                 image_transport tf tf_conversions dynamic_reconfigure
  DEPENDS UtiLite
)

###########
## Build ##
###########

MESSAGE(STATUS "UtiLite_INCLUDE_DIRS=${UtiLite_INCLUDE_DIRS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${UtiLite_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#image
add_executable(ucamera src/CameraNode.cpp)
add_dependencies(ucamera ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ucamera ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(rgb2ind src/RGB2IndexedNode.cpp)
add_dependencies(rgb2ind ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rgb2ind ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(rgb2mono src/RGB2MonoNode.cpp)
add_dependencies(rgb2mono ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rgb2mono ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(xy2polar src/Cartesian2PolarNode.cpp)
add_dependencies(xy2polar ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xy2polar ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(motion_filter src/MotionFilterNode.cpp)
add_dependencies(motion_filter ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(motion_filter ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(canny_edge_detector src/CannyEdgeDetectorNode.cpp)
target_link_libraries(canny_edge_detector ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

QT4_WRAP_CPP(moc_srcs src/ImageViewQt.hpp)
add_executable(image_view_qt src/ImageViewQtNode.cpp ${moc_srcs})
target_link_libraries(image_view_qt ${QT_LIBRARIES} ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

# audio
add_executable(recorder src/AudioRecorderNode.cpp)
target_link_libraries(recorder ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(save src/AudioSaveNode.cpp)
add_dependencies(save utilite_generate_messages_cpp)
target_link_libraries(save ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(overlap src/AudioOverlapNode.cpp)
target_link_libraries(overlap ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})

add_executable(player src/AudioPlayerNode.cpp)
target_link_libraries(player ${Fmodex_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${UtiLite_LIBRARIES})
