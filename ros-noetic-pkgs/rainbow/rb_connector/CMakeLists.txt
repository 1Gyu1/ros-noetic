cmake_minimum_required(VERSION 3.2)
project(rb_connector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

find_package(Qt5 COMPONENTS Core REQUIRED)

add_message_files(
  FILES
  rb_data.msg
  rb_command.msg
)

generate_messages(DEPENDENCIES std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib_msgs
    actionlib
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(rb_connector src/rb_connector.cpp)
target_link_libraries(rb_connector ${catkin_LIBRARIES} Qt5::Core)

add_dependencies(rb_connector rb_connector_generate_messages_cpp)


