#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('korus_smach')
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
import tf

import geometry_msgs.msg as geometry_msgs
import manipulation_msgs.msg as manipulation_msgs
import moveit_msgs.msg as moveit_msgs
import pick_and_place_msgs.msg as pick_and_place_msgs
import sensor_msgs.msg as sensor_msgs



class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['prepared'],
                             input_keys=['object_pose',
                                         'tf_listener'],
                             output_keys=['object_pose'])
    def execute(self, userdata):
#        rospy.loginfo('Object pose before adaption:')
#        rospy.loginfo(userdata.object_pose)
        angle = math.atan2(userdata.object_pose.pose.position.y, userdata.object_pose.pose.position.x)
        dist = math.sqrt(math.pow(userdata.object_pose.pose.position.x, 2)
                         + math.pow(userdata.object_pose.pose.position.y, 2))
#        userdata.object_pose.pose.position.x = dist * math.cos(angle)
#        userdata.object_pose.pose.position.y = dist * math.sin(angle)
        yaw = angle
        roll = math.pi / 2
        pitch = 0.0
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        userdata.object_pose.pose.orientation = geometry_msgs.Quaternion(*quat)
#        rospy.loginfo('Object pose after adaption:')
#        rospy.loginfo(userdata.object_pose)
        return 'prepared'

@smach.cb_interface(input_keys=['object_name',
                                'object_pose'])
def pickGoalCb(userdata, goal):
    goal.target_name = userdata.object_name
    goal.group_name = "arm"
    goal.end_effector = "gripper"
    grasp = manipulation_msgs.Grasp()
    grasp.id = "front_grasp"
    grasp.pre_grasp_posture.header.stamp = rospy.Time.now()
    grasp.pre_grasp_posture.header.frame_id = "gripper_link"
    grasp.pre_grasp_posture.name.append("gripper")
    grasp.pre_grasp_posture.position.append(1.3)
    grasp.pre_grasp_posture.velocity.append(0.0)
    grasp.pre_grasp_posture.effort.append(0.0)
#    gripper_state = sensor_msgs.JointState()
    grasp.grasp_posture.header.stamp = grasp.pre_grasp_posture.header.stamp
    grasp.grasp_posture.header.frame_id = grasp.pre_grasp_posture.header.frame_id
    grasp.grasp_posture.name.append("gripper")
    grasp.grasp_posture.position.append(0.2)
    grasp.grasp_posture.velocity.append(0.0)
    grasp.grasp_posture.effort.append(0.0)
    grasp.grasp_pose = userdata.object_pose
    print 'grasp pose'
    print grasp.grasp_pose
    grasp.grasp_quality = 1.0
    gripper_trans_approach = manipulation_msgs.GripperTranslation()
    gripper_trans_approach.direction.header.stamp = grasp.pre_grasp_posture.header.stamp
    gripper_trans_approach.direction.header.frame_id = "palm_link"
    gripper_trans_approach.direction.vector.x = 1.0
    gripper_trans_approach.direction.vector.y = 0.0
    gripper_trans_approach.direction.vector.z = 0.0
    gripper_trans_approach.desired_distance = 0.10
    gripper_trans_approach.min_distance = 0.08
    grasp.approach = gripper_trans_approach
    gripper_trans_retreat = manipulation_msgs.GripperTranslation()
    gripper_trans_retreat.direction.header = gripper_trans_approach.direction.header
    gripper_trans_retreat.direction.vector.x = 0.0
    gripper_trans_retreat.direction.vector.y = 1.0
    gripper_trans_retreat.direction.vector.z = 0.0
    gripper_trans_retreat.desired_distance = 0.07
    gripper_trans_retreat.min_distance = 0.05
    grasp.retreat = gripper_trans_retreat
    grasp.max_contact_force = 0.0 # disabled
#    grasp.allowed_touch_objects.append("all") # optional
    goal.possible_grasps.append(grasp)
    goal.allow_gripper_support_collision = False
    goal.attached_object_touch_links = ['finger_left_knuckle_1_link',
                                        'finger_left_knuckle_2_link',
                                        'finger_left_tip_link',
                                        'finger_right_knuckle_1_link',
                                        'finger_right_knuckle_2_link',
                                        'finger_right_tip_link']
    goal.minimize_object_distance = False
#    goal.path_constraints = ...
    goal.planner_id = ""
    goal.allowed_touch_objects.append("all")
    goal.allowed_planning_time = 20.0
#    goal.planning_options = ...
    return goal

def pickResultCb(userdata, status, result):
    rospy.loginfo('Pickup status: ' + str(status))
#    rospy.loginfo('Pickup result:')
#    rospy.loginfo(result)
    return 'succeeded'

def createSM():
    
    sm = smach.StateMachine(outcomes=['picked',
                                      'pick_failed',
                                      'preempted',
                                      'error'],
                            input_keys=['object_name',
                                        'object_pose'],
                            output_keys=['object_pose'])

    with sm:
        
        sm.userdata.pose_arm_default = geometry_msgs.PoseStamped()
        sm.userdata.pose_arm_default.header.stamp = rospy.Time.now()
        sm.userdata.pose_arm_default.header.frame_id = "/base_footprint"
        sm.userdata.pose_arm_default.pose.position.x = 0.05
        sm.userdata.pose_arm_default.pose.position.y = 0.0
        sm.userdata.pose_arm_default.pose.position.z = 0.50
        sm.userdata.pose_arm_default.pose.orientation.w = 1.0
        
        smach.StateMachine.add('Prepare',
                               Prepare(),
                               remapping={'object_pose':'object_pose'},
                               transitions={'prepared':'PickObject'})

        smach.StateMachine.add('PickObject',
                               SimpleActionState('pickup',
                                                 moveit_msgs.PickupAction,
                                                 goal_cb=pickGoalCb,
                                                 result_cb=pickResultCb),
                               remapping={'object_name':'object_name',
                                          'object_pose':'object_pose'},
                               transitions={'succeeded':'MoveArmToDefaultSuccess',
                                            'aborted':'MoveArmToDefaultAborted',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveArmToDefaultSuccess',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'picked',
                                            'aborted':'MoveArmToDefaultTryAgain',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveArmToDefaultAborted',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'pick_failed',
                                            'aborted':'MoveArmToDefaultTryAgain',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveArmToDefaultTryAgain',
                               SimpleActionState('move_arm_planner',
                                                 pick_and_place_msgs.MoveArmAction,
                                                 goal_slots=['goal_pose']),
                               remapping={'goal_pose':'pose_arm_default'},
                               transitions={'succeeded':'error',
                                            'aborted':'error',
                                            'preempted':'preempted'})
        
    return sm
