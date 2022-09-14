cmake_minimum_required(VERSION 3.5)
project(rosee_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
#find_package(end_effector REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosee_msg REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#not necessary anymore, we set the path as cmake flag
#set(QT5_PATH "/usr/share/qt5-15/5.15.0/gcc_64") #iit pc
#set(QT5_PATH "/usr/lib/x86_64-linux-gnu/Qt5.12.8/5.12.8/gcc_64") #tori pc

#set(QT5_PATH "" CACHE PATH "Path to QT > 5.9")
#set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${QT5_PATH}")

#if(CMAKE_VERSION VERSION_LESS "3.7.0")
#    set(CMAKE_INCLUDE_CURRENT_DIR ON)
#endif()

#First one is necessary because even if we have a qt version greater than 5.9,
#if we use only the second find package the variable ${Qt5_VERSION} remains 5.5
#find_package(Qt5 5.9 COMPONENTS Widgets)
#find_package(Qt5 REQUIRED COMPONENTS Widgets REQUIRED )
#find_package(Qt5Charts 5.9 QUIET)
#find_package(Qt5UiTools 5.9 QUIET)
#find_package(Qt5Widgets REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
)

#VERSION_GREATER_EQUAL require cmake 3.7, we are using minimum 3.1
#if ( Qt5UiTools_FOUND AND Qt5Charts_FOUND )
#    set(SECOND_TAB ON)
#    add_definitions(-DSECOND_TAB_CODE=1)
#    message("Compiling also the 2nd tab")
#else()
    set(SECOND_TAB OFF)
    add_definitions(-DSECOND_TAB_CODE=0)
    message("Compiling without the 2nd tab")
#endif()

#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(SECOND_TAB)
    add_subdirectory(src/chart/)
    add_subdirectory(src/joint_state_gui/)
endif()

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    #${Qt5Core_INCLUDE_DIRS}
   # ${Qt5Widgets_INCLUDE_DIRS}
)

set(QT_LIBRARIES
    Qt5::Core
    Qt5::Widgets
)

set(MSG_LIBS
    rosee_msg
    rclcpp_action
    action_msgs
)

## Declare a C++ library
# needed also the .h for qt automoc things

list(APPEND libToInstall)

add_library(${PROJECT_NAME}_MainWindow SHARED
    src/MainWindow.cpp
    include/rosee_gui/MainWindow.h
)
list(APPEND libToInstall ${PROJECT_NAME}_MainWindow)

add_library(${PROJECT_NAME}_TabAction SHARED
    src/TabAction.cpp
    include/rosee_gui/TabAction.h
)
list(APPEND libToInstall ${PROJECT_NAME}_TabAction )

add_library(${PROJECT_NAME}_SingleActionGroupBox SHARED
    src/SingleActionGroupBox.cpp
    include/rosee_gui/SingleActionGroupBox.h
)
list(APPEND libToInstall ${PROJECT_NAME}_SingleActionGroupBox)

add_library(${PROJECT_NAME}_SingleActionBoxesGroupBox SHARED
    src/SingleActionBoxesGroupBox.cpp
    include/rosee_gui/SingleActionBoxesGroupBox.h
)
list(APPEND libToInstall ${PROJECT_NAME}_SingleActionBoxesGroupBox )

add_library(${PROJECT_NAME}_SingleActionTimedGroupBox SHARED
    src/SingleActionTimedGroupBox.cpp
    include/rosee_gui/SingleActionTimedGroupBox.h
)
list(APPEND libToInstall ${PROJECT_NAME}_SingleActionTimedGroupBox )

add_library(${PROJECT_NAME}_ActionTimedElement SHARED
    src/ActionTimedElement.cpp
    include/rosee_gui/ActionTimedElement.h
)
list(APPEND libToInstall ${PROJECT_NAME}_ActionTimedElement )

add_library(${PROJECT_NAME}_TimerHandler SHARED
    src/TimerHandler.cpp
    include/rosee_gui/TimerHandler.h
)
list(APPEND libToInstall ${PROJECT_NAME}_TimerHandler)

add_library(${PROJECT_NAME}_JointStateTable SHARED
    src/JointStateTable.cpp
    include/rosee_gui/JointStateTable.h
)
list(APPEND libToInstall ${PROJECT_NAME}_JointStateTable)

add_library(${PROJECT_NAME}_JointStateContainer SHARED
    src/JointStateContainer.cpp
    include/rosee_gui/JointStateContainer.h 
)
list(APPEND libToInstall ${PROJECT_NAME}_JointStateContainer)

add_library(${PROJECT_NAME}_ContainerActionGroupBox SHARED
    src/ContainerActionGroupBox.cpp
    include/rosee_gui/ContainerActionGroupBox.h 
)
list(APPEND libToInstall ${PROJECT_NAME}_ContainerActionGroupBox)

add_library(${PROJECT_NAME}_RobotDescriptionHandler SHARED
    src/RobotDescriptionHandler.cpp
    include/rosee_gui/RobotDescriptionHandler.h 
)
list(APPEND libToInstall ${PROJECT_NAME}_RobotDescriptionHandler)


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
if(SECOND_TAB)
    list(APPEND 2tabLibs "joint_bar_ui")
endif()

ament_target_dependencies(${PROJECT_NAME}_gui_main
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_gui_main
    Qt5::Widgets
    ${PROJECT_NAME}_MainWindow
    ${PROJECT_NAME}_TimerHandler
)

ament_target_dependencies(${PROJECT_NAME}_MainWindow
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_MainWindow
    Qt5::Widgets
    ${PROJECT_NAME}_TabAction
    ${PROJECT_NAME}_RobotDescriptionHandler
    ${2tabLibs}
)

ament_target_dependencies(${PROJECT_NAME}_TabAction
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_TabAction
    #Qt5::Widgets
    ${PROJECT_NAME}_ContainerActionGroupBox
    ${PROJECT_NAME}_JointStateContainer
    ${PROJECT_NAME}_RobotDescriptionHandler
)


ament_target_dependencies(${PROJECT_NAME}_SingleActionGroupBox
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_SingleActionGroupBox
    Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}_SingleActionBoxesGroupBox
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_SingleActionBoxesGroupBox
    Qt5::Widgets
    ${PROJECT_NAME}_SingleActionGroupBox
)

ament_target_dependencies(${PROJECT_NAME}_SingleActionTimedGroupBox
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_SingleActionTimedGroupBox
    Qt5::Widgets
    ${PROJECT_NAME}_ActionTimedElement
)

ament_target_dependencies(${PROJECT_NAME}_ActionTimedElement
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_ActionTimedElement
    Qt5::Widgets
   
)

ament_target_dependencies(${PROJECT_NAME}_TimerHandler
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_TimerHandler
    Qt5::Core
    
)

ament_target_dependencies(${PROJECT_NAME}_JointStateTable
    rclcpp
    ${MSG_LIBS}
    sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_JointStateTable
    Qt5::Widgets
    ${PROJECT_NAME}_RobotDescriptionHandler

    
)

ament_target_dependencies(${PROJECT_NAME}_JointStateContainer
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_JointStateContainer
    Qt5::Widgets
    ${PROJECT_NAME}_JointStateTable
    ${PROJECT_NAME}_RobotDescriptionHandler
    
)
ament_target_dependencies(${PROJECT_NAME}_ContainerActionGroupBox
    rclcpp
    ${MSG_LIBS}
)
target_link_libraries(${PROJECT_NAME}_ContainerActionGroupBox
    Qt5::Widgets
    ${PROJECT_NAME}_SingleActionGroupBox
    ${PROJECT_NAME}_SingleActionBoxesGroupBox
    ${PROJECT_NAME}_SingleActionTimedGroupBox
    
)
ament_target_dependencies(${PROJECT_NAME}_RobotDescriptionHandler
    rclcpp
    urdf
    srdfdom
    ${MSG_LIBS}
)

#############
## Install ##
#############
# Mark executables and/or libraries for installation
install(TARGETS 
    ${PROJECT_NAME}_gui_main
    ${libToInstall}
    DESTINATION lib/${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

install(DIRECTORY launch/
  DESTINATION launch
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_export_libraries(
  rosee_gui
)
ament_export_targets(
  export_${PROJECT_NAME}
)


ament_package()
