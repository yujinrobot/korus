##############################################################################
# Files
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

add_library(korus_rapi ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

##############################################################################
# Installation 
##############################################################################

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})