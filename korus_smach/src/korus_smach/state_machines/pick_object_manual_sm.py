#!/usr/bin/env python
import math

import korus_smach
from korus_smach.pick_and_place_tools import trajectory_control
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
import tf

import control_msgs.msg as control_msgs
import geometry_msgs.msg as geometry_msgs
import manipulation_msgs.msg as manipulation_msgs
import moveit_msgs.msg as moveit_msgs
import pick_and_place_msgs.msg as pick_and_place_msgs




class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=['object_pose',
                                         'pre_grasp_pose',
                                         'grasp_pose',
                                         'post_grasp_pose',
                                         'pre_grasp_dist',
                                         'pre_grasp_height',
                                         'grasp_dist',
                                         'post_grasp_height'],
                             output_keys=['pre_grasp_pose',
                                          'grasp_pose',
                                          'post_grasp_pose'])
    def execute(self, userdata):
        userdata.pre_grasp_pose.header.stamp = rospy.Time.now()
        userdata.pre_grasp_pose.header.frame_id = "/base_footprint"
        userdata.grasp_pose.header = userdata.pre_grasp_pose.header
        userdata.post_grasp_pose.header = userdata.pre_grasp_pose.header
        
        angle = math.atan2(userdata.object_pose.pose.position.y, userdata.object_pose.pose.position.x)
        dist = math.sqrt(math.pow(userdata.object_pose.pose.position.x, 2)
                         + math.pow(userdata.object_pose.pose.position.y, 2))
        yaw = angle
        roll = math.pi / 2
        pitch = 0.0
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        
        userdata.pre_grasp_pose.pose.position.x = (dist - userdata.pre_grasp_dist) * math.cos(angle)
        userdata.pre_grasp_pose.pose.position.y = (dist - userdata.pre_grasp_dist) * math.sin(angle)
        userdata.pre_grasp_pose.pose.position.z = userdata.object_pose.pose.position.z + userdata.pre_grasp_height
        userdata.pre_grasp_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
        userdata.grasp_pose.pose.position.x = (dist - userdata.grasp_dist) * math.cos(angle)
        userdata.grasp_pose.pose.position.y = (dist - userdata.grasp_dist) * math.sin(angle)
        userdata.grasp_pose.pose.position.z = userdata.object_pose.pose.position.z + userdata.pre_grasp_height
        userdata.grasp_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
        userdata.post_grasp_pose.pose.position.x = (dist - userdata.grasp_dist) * math.cos(angle)
        userdata.post_grasp_pose.pose.position.y = (dist - userdata.grasp_dist) * math.sin(angle)
        userdata.post_grasp_pose.pose.position.z = userdata.object_pose.pose.position.z + userdata.post_grasp_height
        userdata.post_grasp_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
        
        rospy.loginfo('Object pose:')
        rospy.loginfo(userdata.object_pose.pose)
        rospy.loginfo('Pre grasp pose:')
        rospy.loginfo(userdata.pre_grasp_pose.pose)
        rospy.loginfo('Grasp pose:')
        rospy.loginfo(userdata.grasp_pose.pose)
        rospy.loginfo('Post grasp pose:')
        rospy.loginfo(userdata.post_grasp_pose.pose)
        return 'prepared'

#@smach.cb_interface(input_keys=['object_name',
#                                'object_pose'])
#def pickGoalCb(userdata, goal):
#    goal.target_name = userdata.object_name
#    goal.group_name = "arm"
#    goal.end_effector = "gripper"
#    grasp = manipulation_msgs.Grasp()
#    grasp.id = "front_grasp"
#    grasp.pre_grasp_posture.header.stamp = rospy.Time.now()
#    grasp.pre_grasp_posture.header.frame_id = "gripper_link"
#    grasp.pre_grasp_posture.name.append("gripper")
#    grasp.pre_grasp_posture.position.append(1.1)
#    grasp.pre_grasp_posture.velocity.append(0.0)
#    grasp.pre_grasp_posture.effort.append(0.0)
##    gripper_state = sensor_msgs.JointState()
#    grasp.grasp_posture.header.stamp = grasp.pre_grasp_posture.header.stamp
#    grasp.grasp_posture.header.frame_id = grasp.pre_grasp_posture.header.frame_id
#    grasp.grasp_posture.name.append("gripper")
#    grasp.grasp_posture.position.append(0.2)
#    grasp.grasp_posture.velocity.append(0.0)
#    grasp.grasp_posture.effort.append(0.0)
#    grasp.grasp_pose = userdata.object_pose
#    print 'grasp pose'
#    print grasp.grasp_pose
#    grasp.grasp_quality = 1.0
#    gripper_trans_approach = manipulation_msgs.GripperTranslation()
#    gripper_trans_approach.direction.header.stamp = grasp.pre_grasp_posture.header.stamp
#    gripper_trans_approach.direction.header.frame_id = "palm_link"
#    gripper_trans_approach.direction.vector.x = 1.0
#    gripper_trans_approach.direction.vector.y = 0.0
#    gripper_trans_approach.direction.vector.z = 0.0
#    gripper_trans_approach.desired_distance = 0.15
#    gripper_trans_approach.min_distance = 0.10
#    grasp.approach = gripper_trans_approach
#    gripper_trans_retreat = manipulation_msgs.GripperTranslation()
#    gripper_trans_retreat.direction.header = gripper_trans_approach.direction.header
#    gripper_trans_retreat.direction.vector.x = -1.0
#    gripper_trans_retreat.direction.vector.y = 0.0
#    gripper_trans_retreat.direction.vector.z = 0.0
#    gripper_trans_retreat.desired_distance = 0.15
#    gripper_trans_retreat.min_distance = 0.10
#    grasp.retreat = gripper_trans_retreat
#    grasp.max_contact_force = 0.0 # disabled
#    grasp.allowed_touch_objects = [] # optional
#    goal.possible_grasps.append(grasp)
#    goal.allow_gripper_support_collision = False
#    goal.attached_object_touch_links = ['finger_left_knuckle_1_link',
#                                        'finger_left_knuckle_2_link',
#                                        'finger_left_tip_link',
#                                        'finger_right_knuckle_1_link',
#                                        'finger_right_knuckle_2_link',
#                                        'finger_right_tip_link']
#    goal.minimize_object_distance = False
##    goal.path_constraints = ...
#    goal.planner_id = ""
#    goal.allowed_touch_objects = [userdata.object_name]
#    goal.allowed_planning_time = 20.0
##    goal.planning_options = ...
#    return goal
#
#def pickResultCb(userdata, status, result):
#    rospy.loginfo('Pickup status: ' + str(status))
##    rospy.loginfo('Pickup result:')
##    rospy.loginfo(result)
#    return 'succeeded'

def createSM():
    
    sm = smach.StateMachine(outcomes=['picked',
                                      'pick_failed',
                                      'preempted'],
                            input_keys=['object_name',
                                        'object_pose',
                                        'pre_grasp_dist',
                                        'pre_grasp_height',
                                        'grasp_dist',
                                        'post_grasp_height',
                                        'angle_gripper_open',
                                        'angle_gripper_closed',
                                        'pose_arm_default'],
                            output_keys=['object_pose'])

    with sm:
        sm.userdata.pre_grasp_pose = geometry_msgs.PoseStamped()
        sm.userdata.grasp_pose = geometry_msgs.PoseStamped()
        sm.userdata.post_grasp_pose = geometry_msgs.PoseStamped()
        sm.userdata.true = True
        sm.userdata.false = False
        
        smach.StateMachine.add('Prepare',
                               Prepare(),
                               remapping={'object_pose':'object_pose',
                                          'pre_grasp_pose':'pre_grasp_pose',
                                          'grasp_pose':'grasp_pose',
                                          'post_grasp_pose':'post_grasp_pose',
                                          'pre_grasp_dist':'pre_grasp_dist',
                                          'pre_grasp_height':'pre_grasp_height',
                                          'grasp_dist':'grasp_dist',
                                          'post_grasp_height':'post_grasp_height'},
                               transitions={'prepared':'MoveArmPreGrasp'})
        
        smach.StateMachine.add('MoveArmPreGrasp',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pre_grasp_pose'},
                               transitions={'succeeded':'OpenGripper',
                                            'aborted':'pick_failed',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('OpenGripper',
                               SimpleActionState('gripper_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.gripperControlGoalCb,
                                                 result_cb=trajectory_control.gripperControlResultCb),
                               remapping={'angle':'angle_gripper_open',
                                          'open_gripper':'true',
                                          'close_gripper':'false'},
                               transitions={'succeeded':'CloseGripper',
                                            'aborted':'pick_failed',
                                            'preempted':'preempted'})
        
#        smach.StateMachine.add('MoveArmIKGrasp',
#                               SimpleActionState('move_arm_planner',
#                                                 pick_and_place_msgs.MoveArmAction,
#                                                 goal_slots=['goal_pose']),
#                               remapping={'goal_pose':'pose_arm_default'},
#                               transitions={'succeeded':'picked',
#                                            'aborted':'MoveArmToDefaultTryAgain',
#                                            'preempted':'preempted'})
        
        smach.StateMachine.add('CloseGripper',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'MoveArmDefault',
                                            'aborted':'pick_failed',
                                            'preempted':'preempted'})

#        smach.StateMachine.add('AttachObject',
#                               SimpleActionState('move_arm_planner',
#                                                 pick_and_place_msgs.MoveArmAction,
#                                                 goal_slots=['goal_pose']),
#                               remapping={'goal_pose':'pose_arm_default'},
#                               transitions={'succeeded':'picked',
#                                            'aborted':'MoveArmToDefaultTryAgain',
#                                            'preempted':'preempted'})
        
#        smach.StateMachine.add('MoveArmIKPostGrasp',
#                               SimpleActionState('move_arm_planner',
#                                                 pick_and_place_msgs.MoveArmAction,
#                                                 goal_slots=['goal_pose']),
#                               remapping={'goal_pose':'pose_arm_default'},
#                               transitions={'succeeded':'picked',
#                                            'aborted':'MoveArmToDefaultTryAgain',
#                                            'preempted':'preempted'})

        smach.StateMachine.add('MoveArmDefault',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'picked',
                                            'aborted':'pick_failed',
                                            'preempted':'preempted'})
        
    return sm
