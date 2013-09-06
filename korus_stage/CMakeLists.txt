cmake_minimum_required(VERSION 2.8.3)
project(korus_stage)
catkin_package()

install(DIRECTORY launch
                  maps
                  worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
