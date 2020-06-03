cmake_minimum_required(VERSION 3.1.0)
project(rosee_gui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

## qt5.12 path to be declared
#set(QT5_PATH "/usr/lib/x86_64-linux-gnu/Qt5.12.8/5.12.8/gcc_64")
#set(QT5_PATH "" CACHE PATH "Path to QT > 5.9")
#set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${QT5_PATH}")


if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    ros_end_effector #for action type 
    rosee_msg
    actionlib
    srdfdom
    # rqt integration:
    rqt_gui 
    rqt_gui_cpp
)


#not catkin packages
find_package(Qt5 COMPONENTS Widgets REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(ros_end_effector REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msgs' folder
# add_message_files(
#   DIRECTORY msgs
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    ros_end_effector #for action type 
    rosee_msg
    actionlib
    srdfdom
    # rqt integration:
    rqt_gui 
    rqt_gui_cpp
#  DEPENDS system_lib
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(src/chart/)
add_subdirectory(src/joint_state_gui/)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${ros_end_effector_INCLUDE_DIRS}
)

## Declare a C++ library
# needed also the .h for qt automoc things
add_library(${PROJECT_NAME}_MainWindow SHARED
    src/MainWindow.cpp
    include/rosee_gui/MainWindow.h
)

add_library(${PROJECT_NAME}_TabAction SHARED
    src/TabAction.cpp
    include/rosee_gui/TabAction.h
)
qt5_wrap_cpp(${PROJECT_NAME}_TabAction_MOCS src/TabAction.cpp)

add_library(${PROJECT_NAME}_SingleActionGroupBox SHARED
    src/SingleActionGroupBox.cpp
    include/rosee_gui/SingleActionGroupBox.h
)

add_library(${PROJECT_NAME}_SingleActionBoxesGroupBox SHARED
    src/SingleActionBoxesGroupBox.cpp
    include/rosee_gui/SingleActionBoxesGroupBox.h
)

add_library(${PROJECT_NAME}_SingleActionTimedGroupBox SHARED
    src/SingleActionTimedGroupBox.cpp
    include/rosee_gui/SingleActionTimedGroupBox.h
)

add_library(${PROJECT_NAME}_ActionTimedElement SHARED
    src/ActionTimedElement.cpp
    include/rosee_gui/ActionTimedElement.h
)

add_library(${PROJECT_NAME}_TimerHandler SHARED
    src/TimerHandler.cpp
    include/rosee_gui/TimerHandler.h
)

add_library(${PROJECT_NAME}_JointStateTable SHARED
    src/JointStateTable.cpp
    include/rosee_gui/JointStateTable.h
)

add_library(${PROJECT_NAME}_JointStateContainer SHARED
    src/JointStateContainer.cpp
    include/rosee_gui/JointStateContainer.h 
)

add_library(${PROJECT_NAME}_ContainerActionGroupBox SHARED
    src/ContainerActionGroupBox.cpp
    include/rosee_gui/ContainerActionGroupBox.h 
)

add_library(${PROJECT_NAME}_RobotDescriptionHandler SHARED
    src/RobotDescriptionHandler.cpp
    include/rosee_gui/RobotDescriptionHandler.h 
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_gui_main src/main.cpp)
set_target_properties(${PROJECT_NAME}_gui_main PROPERTIES OUTPUT_NAME "gui_main")

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_gui_main
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${PROJECT_NAME}_MainWindow
    ${PROJECT_NAME}_TimerHandler
    joint_bar_ui

)

target_link_libraries(${PROJECT_NAME}_MainWindow
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${PROJECT_NAME}_TabAction
    ${PROJECT_NAME}_RobotDescriptionHandler

)

target_link_libraries(${PROJECT_NAME}_TabAction
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${PROJECT_NAME}_ContainerActionGroupBox
    ${PROJECT_NAME}_JointStateContainer
    ${PROJECT_NAME}_RobotDescriptionHandler
)


target_link_libraries(${PROJECT_NAME}_SingleActionGroupBox
    ${catkin_LIBRARIES}
    Qt5::Widgets
)

target_link_libraries(${PROJECT_NAME}_SingleActionBoxesGroupBox
    ${catkin_LIBRARIES}
    Qt5::Widgets
)

target_link_libraries(${PROJECT_NAME}_SingleActionTimedGroupBox
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${PROJECT_NAME}_ActionTimedElement
)

target_link_libraries(${PROJECT_NAME}_ActionTimedElement
    ${catkin_LIBRARIES}
    Qt5::Widgets
)

target_link_libraries(${PROJECT_NAME}_TimerHandler
    ${catkin_LIBRARIES}
    Qt5::Widgets
)

target_link_libraries(${PROJECT_NAME}_JointStateTable
    ${catkin_LIBRARIES}
    Qt5::Widgets
)

target_link_libraries(${PROJECT_NAME}_JointStateContainer
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${PROJECT_NAME}_JointStateTable
    ${PROJECT_NAME}_RobotDescriptionHandler
)
target_link_libraries(${PROJECT_NAME}_ContainerActionGroupBox
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${PROJECT_NAME}_SingleActionGroupBox
    ${PROJECT_NAME}_SingleActionBoxesGroupBox
    ${PROJECT_NAME}_SingleActionTimedGroupBox
)
target_link_libraries(${PROJECT_NAME}_RobotDescriptionHandler
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
 
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}_gui_main
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosee_gui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
