cmake_minimum_required(VERSION 3.0)

project(dvrk_record)
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Find OpenCV
find_package( OpenCV REQUIRED )

# Find QT
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Widgets REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES tcpwrap_qt
		#  CATKIN_DEPENDS roscpp rospy std_msgs
		#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Set your files and resources here
file(GLOB Headers include/*.h include/*.hpp)
file(GLOB Sources src/*.cpp)

file(GLOB RC_FILES *.qrc)
file(GLOB UI_FILES src/*.ui)
qt5_wrap_ui(UISrcs ${UI_FILES} )

## Declare a cpp executable
add_executable(dvrk_record_node MACOSX_BUNDLE
		${Sources} 
		${Headers} 
		${UISrcs}
		${RC_FILES}
		)

## Specify libraries to link a library or executable target against
target_link_libraries(dvrk_record_node
    Qt5::Widgets
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)