##############################################################################
# Files
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

add_executable(korus_rapi_node ${SOURCES})
target_link_libraries(korus_rapi_node korus_rapi)
