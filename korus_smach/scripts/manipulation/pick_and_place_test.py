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
from korus_smach.msg import MoveArmAction



def main():
    rospy.init_node('pick_and_place')

    sm = smach.StateMachine(outcomes=['success', 'error'])
    with sm:
        sm.userdata.wait_0sec = 0.0
        # userdata for find object
        sm.userdata.table_left_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.table_left_pose.header.stamp = rospy.Time.now()
        sm.userdata.table_left_pose.header.frame_id = "/base_footprint"
        sm.userdata.table_left_pose.pose.position.x = 0.50
        sm.userdata.table_left_pose.pose.position.y = 0.50
        sm.userdata.table_left_pose.pose.position.z = 0.30
        sm.userdata.table_left_pose.pose.orientation.x = 0.0
        sm.userdata.table_left_pose.pose.orientation.y = 0.0
        sm.userdata.table_left_pose.pose.orientation.z = 0.0
        sm.userdata.table_left_pose.pose.orientation.w = 1.0
        sm.userdata.table_right_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.table_right_pose.header = sm.userdata.table_left_pose.header
        sm.userdata.table_right_pose.pose.position.x = 0.30
        sm.userdata.table_right_pose.pose.position.y = -0.40
        sm.userdata.table_right_pose.pose.position.z = 0.40
        sm.userdata.table_right_pose.pose.orientation = sm.userdata.table_left_pose.pose.orientation
        sm.userdata.table_centre_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.table_centre_pose.header = sm.userdata.table_left_pose.header
        sm.userdata.table_centre_pose.pose.position.x = 0.80
        sm.userdata.table_centre_pose.pose.position.y = 0.0
        sm.userdata.table_centre_pose.pose.position.z = 0.65
        sm.userdata.table_centre_pose.pose.orientation = sm.userdata.table_left_pose.pose.orientation
        
        sm.userdata.table_left_position = geometry_msgs.msg.PointStamped()
        sm.userdata.table_left_position.header = sm.userdata.table_left_pose.header
        sm.userdata.table_left_position.point.x =  sm.userdata.table_left_pose.pose.position.x
        sm.userdata.table_left_position.point.y =  sm.userdata.table_left_pose.pose.position.y
        sm.userdata.table_left_position.point.z =  sm.userdata.table_left_pose.pose.position.z
        
        
        sm.userdata.test_pose = geometry_msgs.msg.PoseStamped()
        sm.userdata.test_pose.header.stamp = rospy.Time.now()
        sm.userdata.test_pose.header.frame_id = "base_footprint"
        sm.userdata.test_pose.pose.position.x = 0.5
        sm.userdata.test_pose.pose.position.y = 0.2
        sm.userdata.test_pose.pose.position.z = 0.5
        sm.userdata.test_pose.pose.orientation.x = 0.0
        sm.userdata.test_pose.pose.orientation.y = 0.0
        sm.userdata.test_pose.pose.orientation.z = 0.0
        sm.userdata.test_pose.pose.orientation.w = 1.0
        sm.userdata.wait_1sec = 1.0
        sm.userdata.wait_2sec = 2.0
        
        sm.userdata.attach_true = True
        sm.userdata.attach_false = False
        sm.userdata.object_name = "test_object"
        
        smach.StateMachine.add('Starter',
                               misc_tools.Wait(),
                               remapping={'duration':'wait_1sec'},
                               transitions={'done':'MoveArmLeft'})
        
        smach.StateMachine.add('MoveArmLeft',
                       SimpleActionState('/korus/move_arm_planner',
                                         MoveArmAction,
                                         goal_slots=['goal_pose']),
                       remapping={'goal_pose':'table_left_pose'},
                       transitions={'succeeded':'AttachObject',
                                    'aborted':'ClearCollisionObjects',
                                    'preempted':'error'})
        
#        smach.StateMachine.add('AttachObject',
#                               ServiceState('environment_server/set_planning_scene_diff',
#                                            arm_navigation_msgs.srv.SetPlanningSceneDiff,
#                                            request_cb = misc_tools.attachObjectRequestCb,
#                                            input_keys=['object_name',
#                                                        'attach']),
#                               remapping={'object_name':'object_name',
#                                          'attach':'attach_true'},
#                               transitions={'succeeded':'MoveArmRight',
#                                            'preempted':'error',
#                                            'aborted':'error'})
        
        StateMachine.add('AttachObject', 
                         misc_tools.ManipulateCollisionObject(),
                         remapping={'object_name':'object_name',
                                    'attach':'attach_true'},
                         transitions={'done':'MoveArmRight'})
        
        
        smach.StateMachine.add('MoveArmRight',
               SimpleActionState('/korus/move_arm_planner',
                                 MoveArmAction,
                                 goal_slots=['goal_pose']),
               remapping={'goal_pose':'table_right_pose'},
               transitions={'succeeded':'ClearCollisionObjects',
                            'aborted':'ClearCollisionObjects',
                            'preempted':'error'})
        

#        StateMachine.add('DetachObject', 
#                         misc_tools.ManipulateCollisionObject(),
#                         remapping={'object_name':'object_name',
#                                    'attach':'attach_false'},
#                         transitions={'done':'MoveArmDefault'})

        StateMachine.add('ClearCollisionObjects', 
                         misc_tools.ClearCollisionObjects(),
                         transitions={'done':'Starter'})
#        
#        smach.StateMachine.add('MoveArmDefault',
#               SimpleActionState('/korus/move_arm_planner',
#                                 MoveArmAction,
#                                 goal_slots=['goal_pose']),
#               remapping={'goal_pose':'test_pose'},
#               transitions={'succeeded':'success',
#                            'aborted':'error',
#                            'preempted':'error'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Stop introspection server after state machine finishes
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

