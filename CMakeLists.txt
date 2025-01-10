cmake_minimum_required(VERSION 2.8.3)
project(mujoco_ros_control)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-std=c++11)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED 
             roscpp 
             roslib 
             hardware_interface 
             controller_manager
             common_robot_functions)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

find_package(OpenGL REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${mujoco_INCLUDE_DIR} ${glfw3_INCLUDE_DIR})

add_executable(mujoco_control src/mujoco_control.cpp src/RobotHWMujoco.cpp)
add_dependencies(mujoco_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mujoco_control ${catkin_LIBRARIES} ${mujoco_LIBRARIES} ${YAML_CPP_LIBRARIES} common_robot_functions::common_robot_functions_transformations common_robot_functions::common_robot_functions_mujoco)


install(TARGETS mujoco_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)

if(CATKIN_ENABLE_TESTING)
    add_subdirectory(tests)
endif()
