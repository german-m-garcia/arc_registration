cmake_minimum_required(VERSION 2.8.3)
project(arc_registration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-Wall -g  -lpthread -std=c++0x -O4) #-std=c++11 -march=native

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rviz
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation  
  tf_conversions
  eigen_conversions
  rosmrsmap
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package( Boost COMPONENTS filesystem system REQUIRED)


if(${rviz_VERSION} VERSION_LESS 1.12)
        message(STATUS "RViz version is < 1.12 (=> Indigo), using Qt4")
        find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
        include(${QT_USE_FILE})
        include_directories(${CMAKE_CURRENT_BINARY_DIR})
        qt4_wrap_ui(UI_SRCS 
                 include/arc_registration/grasp_editor.ui
        )
        qt4_wrap_cpp(MOC_SRCS
                 include/arc_registration/grasp_editor.h
        )
else()
        message(STATUS "RViz version is >= 1.12 (=> Kinetic), using Qt5")
        find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
        include_directories(${CMAKE_CURRENT_BINARY_DIR})
        qt5_wrap_ui(UI_SRCS
                 include/arc_registration/grasp_editor.ui
        )
        qt5_wrap_cpp(MOC_SRCS
                 include/arc_registration/grasp_editor.h
        )
endif()


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Registration.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES  geometry_msgs sensor_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arc_registration
  #CATKIN_DEPENDS roscpp rospy arc_rendering message_runtime
  CATKIN_DEPENDS roscpp rospy  message_runtime rosmrsmap
#  DEPENDS system_lib
)

set(CMAKE_AUTORCC on)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)




## Declare a C++ library
add_library(grasp_editor
 ${UI_SRCS}
 ${MOC_SRCS} 
 src/grasp_editor.cpp
 src/object_grasp.cpp 
 src/arc_registration_node.cpp
 resource/icons.qrc
)


target_link_libraries(grasp_editor
        ${catkin_LIBRARIES}
        ${QT_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
)

add_dependencies(grasp_editor ${PROJECT_NAME}_generate_messages_cpp)

add_library(arc_registration
    src/arc_registration_node.cpp
    )

target_link_libraries(arc_registration
    grasp_editor
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
)

add_executable(${PROJECT_NAME}_node src/arc_registration_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
     grasp_editor
     ${catkin_LIBRARIES}
     ${QT_LIBRARIES}
     ${Boost_LIBRARIES}
     ${OpenCV_LIBRARIES}
     ${PCL_LIBRARIES}
     yaml-cpp
 )

add_executable(align_tfs_views src/align_tfs_views.cpp)

 target_link_libraries(align_tfs_views
     grasp_editor
     ${catkin_LIBRARIES}
     ${QT_LIBRARIES}
     ${Boost_LIBRARIES}
     ${OpenCV_LIBRARIES}
     ${PCL_LIBRARIES}
     yaml-cpp
 )

add_executable(register_views src/register_views.cpp)

 target_link_libraries(register_views
     grasp_editor
     ${catkin_LIBRARIES}
     ${QT_LIBRARIES}
     ${Boost_LIBRARIES}
     ${OpenCV_LIBRARIES}
     ${PCL_LIBRARIES}
     yaml-cpp
 )



