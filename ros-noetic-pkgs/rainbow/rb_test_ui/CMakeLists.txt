##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.2)
project(rb_test_ui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  rb_data.msg
  rb_command.msg
)

generate_messages(DEPENDENCIES std_msgs)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${catkin_INCLUDE_DIRS}
  )
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
)

##############################################################################
# Qt Environment
##############################################################################





find_package(Qt5 COMPONENTS Core Gui Network Widgets REQUIRED)


# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#$ rosbuild_prepare_qt4(QtCore QtGui QtNetwork) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.hpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(rb_test_ui
  ${QT_SOURCES}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
  ${QT_RESOURCES_CPP}
)

target_link_libraries(rb_test_ui ${catkin_LIBRARIES} Qt5::Gui Qt5::Core Qt5::Network Qt5::Widgets)
install(TARGETS rb_test_ui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

