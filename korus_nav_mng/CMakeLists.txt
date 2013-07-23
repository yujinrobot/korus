# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(korus_nav_mng)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS sensor_board_msgs stargazer_msgs actionlib_msgs geometry_msgs nav_msgs std_msgs roscpp)
find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS sensor_board_msgs stargazer_msgs actionlib_msgs geometry_msgs nav_msgs std_msgs roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# common commands for building c++ executables and libraries
add_executable(navigation_mng src/nav_mng_node.cpp)
add_executable(board_emulator src/nav_mng_test.cpp)

add_dependencies(navigation_mng sensor_board_msgs_gencpp stargazer_msgs_gencpp)
add_dependencies(board_emulator sensor_board_msgs_gencpp stargazer_msgs_gencpp)

target_link_libraries(navigation_mng ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(board_emulator ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS navigation_mng board_emulator
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
