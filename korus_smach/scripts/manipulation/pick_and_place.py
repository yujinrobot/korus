#!/usr/bin/env python

# system
import sys
import math
# ros basics& smach
import roslib; roslib.load_manifest('korus_smach')
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ServiceState

# own lib
from korus_smach.pick_and_place_tools.pick_and_place_imports import *
from korus_smach.pick_and_place_tools import misc_tools
from korus_smach.state_machines import find_object_sm, build_collision_map_sm, pick_sm, place_sm
from korus_smach.msg import FindObjectAction


def main():
    rospy.init_node('pick_and_place')

    sm = smach.StateMachine(outcomes=['success', 'error'])

#    sm.userdata.object_pose = geometry_msgs.msg.PoseStamped()
#    sm.userdata.pre_grasp = False
#    sm.userdata.grasp = False
#    sm.userdata.post_grasp = False
#    
#    sm.userdata.open_gripper_true
    
    # Open the container
    with sm:
        sm.userdata.wait_0sec = 0.0
        # userdata for find object
        sm.userdata.table_left_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.table_left_pose.header.stamp = rospy.Time.now()
        sm.userdata.table_left_pose.header.frame_id = "/base_footprint"
        sm.userdata.table_left_pose.pose.position.x = 0.50
        sm.userdata.table_left_pose.pose.position.y = 0.50
        sm.userdata.table_left_pose.pose.position.z = 0.25
        sm.userdata.table_left_pose.pose.orientation.x = 0.0
        sm.userdata.table_left_pose.pose.orientation.y = 0.0
        sm.userdata.table_left_pose.pose.orientation.z = 0.0
        sm.userdata.table_left_pose.pose.orientation.w = 1.0
        sm.userdata.table_right_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.table_right_pose.header = sm.userdata.table_left_pose.header
        sm.userdata.table_right_pose.pose.position.x = 0.60
        sm.userdata.table_right_pose.pose.position.y = -0.60
        sm.userdata.table_right_pose.pose.position.z = 0.40
        sm.userdata.table_right_pose.pose.orientation = sm.userdata.table_left_pose.pose.orientation
        sm.userdata.table_centre_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.table_centre_pose.header = sm.userdata.table_left_pose.header
        sm.userdata.table_centre_pose.pose.position.x = 0.80
        sm.userdata.table_centre_pose.pose.position.y = 0.0
        sm.userdata.table_centre_pose.pose.position.z = 0.50
        sm.userdata.table_centre_pose.pose.orientation = sm.userdata.table_left_pose.pose.orientation
        
        sm.userdata.table_left_position = geometry_msgs.msg.PointStamped()
        sm.userdata.table_left_position.header = sm.userdata.table_left_pose.header
        sm.userdata.table_left_position.point.x =  sm.userdata.table_left_pose.pose.position.x
        sm.userdata.table_left_position.point.y =  sm.userdata.table_left_pose.pose.position.y
        sm.userdata.table_left_position.point.z =  sm.userdata.table_left_pose.pose.position.z
        sm.userdata.table_right_position = geometry_msgs.msg.PointStamped()
        sm.userdata.table_right_position.header = sm.userdata.table_right_pose.header
        sm.userdata.table_right_position.point.x =  sm.userdata.table_right_pose.pose.position.x
        sm.userdata.table_right_position.point.y =  sm.userdata.table_right_pose.pose.position.y
        sm.userdata.table_right_position.point.z =  sm.userdata.table_right_pose.pose.position.z
        
        sm.userdata.test_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.test_pose.header.stamp = rospy.Time.now()
        sm.userdata.test_pose.header.frame_id = "base_footprint"
        sm.userdata.test_pose.pose.position.x = 0.8
        sm.userdata.test_pose.pose.position.y = 0.0
        sm.userdata.test_pose.pose.position.z = 0.5
        sm.userdata.test_pose.pose.orientation.x = 0.0
        sm.userdata.test_pose.pose.orientation.y = 0.0
        sm.userdata.test_pose.pose.orientation.z = 0.0
        sm.userdata.test_pose.pose.orientation.w = 1.0
        sm.userdata.wait_1sec = 1.0
        sm.userdata.wait_2sec = 2.0
        
#        sm.userdata.test_angle = 0.3
#        smach.StateMachine.add('PickChecker',
#                               misc_tools.PickChecker(),
#                               remapping={'desired_gripper_position':'test_angle'},
#                               transitions={'success':'success',
#                                            'error':'error'})
        
        smach.StateMachine.add('Starter',
                               misc_tools.Wait(),
                               remapping={'duration':'wait_1sec'},
                               transitions={'done':'LookForObjectLeft'})

        smach.StateMachine.add('LookForObjectLeft',
                               SimpleActionState('/korus/find_object_left',
                                                 FindObjectAction,
                                                 goal_slots=['table_position'],
                                                 result_slots=['object_pose',
                                                               'object_name',
                                                               'object_type',
                                                               'error_message',
                                                               'error_code']),
                               remapping={'table_position':'table_left_position',
                                          'object_pose':'object_pose',
                                          'object_name':'object_name'},
                               transitions={'succeeded':'BuildCollisionMapPrePickObject',
                                            'aborted':'LookForObjectRight',
                                            'preempted':'Starter'})
        
        smach.StateMachine.add('LookForObjectRight',
                               SimpleActionState('/korus/find_object_right',
                                                 FindObjectAction,
                                                 goal_slots=['table_position'],
                                                 result_slots=['object_pose',
                                                               'object_name',
                                                               'object_type',
                                                               'error_message',
                                                               'error_code']),
                               remapping={'table_position':'table_right_position',
                                          'object_name':'object_name',
                                          'object_pose':'object_pose'},
                               transitions={'succeeded':'BuildCollisionMapPrePickObject',
                                            'aborted':'Starter',
                                            'preempted':'Starter'})

        sm_build_collision_map = build_collision_map_sm.createSM()
        smach.StateMachine.add('BuildCollisionMapPrePickObject',
                               sm_build_collision_map,
                               transitions={'done':'PickObject',
                                            'aborted':'Starter',
                                            'preempted':'error'})
        
        sm_pick = pick_sm.createSM()
        smach.StateMachine.add('PickObject',
                               sm_pick,
                               remapping={'goal_pose':'object_pose',
                                          'object_name':'object_name'},
                               transitions={'holding_object':'PlaceObject',
                                            'at_default':'Starter',
                                            'error':'Starter'})
        
        sm_place = place_sm.createSM()
        smach.StateMachine.add('PlaceObject',
                               sm_place,
                               remapping={'goal_pose':'table_centre_pose',
                                          'object_name':'object_name'},
                               transitions={'at_default':'Starter',
                                            'error':'Starter'})
    
    sis = smach_ros.IntrospectionServer('introspection_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

