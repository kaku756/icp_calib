cmake_minimum_required(VERSION 3.0.2)
project(finibo_camera_calib)
find_package(catkin REQUIRED)




#add_compile_options(-std=c++14)
#set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        message_filters
        cv_bridge
        message_generation
        image_transport
        tf2_ros
        tf2_geometry_msgs
        visualization_msgs)

find_package(OpenCV REQUIRED)

# catkin_python_setup()

## Generate services in the 'srv' folder
#add_service_files(
#        FILES
#)

#generate_messages(
#        DEPENDENCIES
#        std_msgs
#        sensor_msgs
#        geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs cv_bridge)

include_directories(include external ${catkin_INCLUDE_DIRS})

#add_executable(example_cplusplus_node src/example_cplusplus_node.cpp)
#add_dependencies(example_cplusplus_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(example_cplusplus_node ${catkin_LIBRARIES} ${OpenCV_LIBS})



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
