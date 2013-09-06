cmake_minimum_required(VERSION 2.8.3)
project(korus_smach)

find_package(catkin REQUIRED COMPONENTS actionlib
                                        actionlib_msgs
                                        control_msgs
                                        device_msgs
                                        geometry_msgs
                                        goo_msgs
                                        manipulation_msgs
                                        moveit_msgs
                                        object_recognition_msgs
                                        pick_and_place_msgs
                                        rospy
                                        sensor_msgs
                                        shape_msgs
                                        smach
                                        smach_ros
                                        std_msgs
                                        std_srvs
                                        tf
                                        trajectory_msgs)

catkin_python_setup()

catkin_package()

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
