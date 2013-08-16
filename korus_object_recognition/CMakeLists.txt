cmake_minimum_required(VERSION 2.8.3)
project(korus_object_recognition)

find_package(catkin REQUIRED COMPONENTS korus_meta
                                        object_recognition_ros
                                        object_recognition_tabletop)

catkin_package()

install(DIRECTORY launch
                  config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})