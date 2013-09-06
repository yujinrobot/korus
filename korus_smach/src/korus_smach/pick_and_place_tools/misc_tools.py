#!/usr/bin/env python

import roslib; roslib.load_manifest('korus_smach')
from korus_smach.state_machines.state_machines_imports import *
from korus_smach.pick_and_place_tools.msg_imports import *

#===============================================================================================================
# Add object to robot
#===============================================================================================================
@smach.cb_interface(input_keys=['object_name', 'attach'])
def attachObjectRequestCb(userdata, request):
    if userdata.attach:
        rospy.loginfo("Preparing to attach object ...")
    else:
        rospy.loginfo("Preparing to detach object ...")
    attached_object = arm_navigation_msgs.msg.AttachedCollisionObject()
    attached_object.link_name = "palm_link"
    attached_object.object.id = userdata.object_name
    
    if userdata.attach:
        attached_object.object.operation.operation = arm_navigation_msgs.msg.CollisionObjectOperation.ADD
    else:
        attached_object.object.operation.operation = arm_navigation_msgs.msg.CollisionObjectOperation.REMOVE
    attached_object.object.header.frame_id = "gripper_center_link"
    attached_object.object.header.stamp = rospy.Time.now()
    shape = arm_navigation_msgs.msg.Shape()
    shape.type = arm_navigation_msgs.msg.Shape.CYLINDER
    shape.dimensions.append(0.10)
    shape.dimensions.append(0.20)
    pose = geometry_msgs.msg.Pose() 
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.x = 1
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    attached_object.object.shapes.append(shape)
    attached_object.object.poses.append(pose)
    attached_object.touch_links.append("lower_arm_link");
    attached_object.touch_links.append("wrist_link");
    attached_object.touch_links.append("palm_link");
    attached_object.touch_links.append("gripper_link");
    attached_object.touch_links.append("gripper_center_link");
    attached_object.touch_links.append("gripper_camera_link");
    attached_object.touch_links.append("finger_left_knuckle_1_link");
    attached_object.touch_links.append("finger_left_knuckle_2_link");
    attached_object.touch_links.append("finger_right_knuckle_1_link");
    attached_object.touch_links.append("finger_right_knuckle_2_link");
    set_planning_scene_diff_req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
    set_planning_scene_diff_req.planning_scene_diff.attached_collision_objects.append(attached_object);
    if userdata.attach:
        rospy.loginfo("Prepared to attach object.");
    else:
        rospy.loginfo("Prepared to detach object.")
    return set_planning_scene_diff_req;

class ManipulateCollisionObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['done'],
                             input_keys=['object_name', 'attach'])

    def execute(self, userdata):
        if userdata.attach:
            rospy.loginfo("Preparing to attach object ...")
        else:
            rospy.loginfo("Preparing to detach object ...")
        pub = rospy.Publisher("attached_collision_object", arm_navigation_msgs.msg.AttachedCollisionObject)
        attached_object = arm_navigation_msgs.msg.AttachedCollisionObject()
        attached_object.link_name = "gripper_center_link"
        attached_object.object.id = userdata.object_name
        if userdata.attach:
            attached_object.object.operation.operation = arm_navigation_msgs.msg.CollisionObjectOperation.ADD
        else:
            attached_object.object.operation.operation = arm_navigation_msgs.msg.CollisionObjectOperation.REMOVE
        attached_object.object.header.frame_id = "gripper_center_link"
        attached_object.object.header.stamp = rospy.Time.now()
        shape = arm_navigation_msgs.msg.Shape()
        shape.type = arm_navigation_msgs.msg.Shape.CYLINDER
        shape.dimensions.append(0.10)
        shape.dimensions.append(0.25)
        pose = geometry_msgs.msg.Pose() 
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 1
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        attached_object.object.shapes.append(shape)
        attached_object.object.poses.append(pose)
        attached_object.touch_links.append("lower_arm_link");
        attached_object.touch_links.append("palm_link");
        attached_object.touch_links.append("gripper_link");
        attached_object.touch_links.append("gripper_center_link");
        attached_object.touch_links.append("gripper_camera_link");
        attached_object.touch_links.append("finger_left_knuckle_1_link");
        attached_object.touch_links.append("finger_left_knuckle_2_link");
        attached_object.touch_links.append("finger_right_knuckle_1_link");
        attached_object.touch_links.append("finger_right_knuckle_2_link");
        pub.publish(attached_object)
        if userdata.attach:
            rospy.loginfo("Attached object.");
        else:
            rospy.loginfo("Detached object.")
        return 'done'

class ClearCollisionObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['done'])
        self._pub = rospy.Publisher("attached_collision_object", arm_navigation_msgs.msg.AttachedCollisionObject, latch=True)
        
    def execute(self, userdata):
        rospy.loginfo("Preparing to remove all attached objects ...")
        attached_object = arm_navigation_msgs.msg.AttachedCollisionObject()
#        attached_object.link_name = str(arm_navigation_msgs.msg.AttachedCollisionObject.REMOVE_ALL_ATTACHED_OBJECTS)
        attached_object.link_name = 'all'
        attached_object.object.id = ""
        attached_object.object.operation.operation = arm_navigation_msgs.msg.CollisionObjectOperation.REMOVE
        self._pub.publish(attached_object)
#        rospy.sleep(3.0) # wait a bit to make sure all subscribers will receive the message
        rospy.loginfo("Removed all attached objects. <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        return 'done'


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['done'],
                             input_keys=['duration'])

    def execute(self, userdata):
        rospy.loginfo('Waiting for ' + str(userdata.duration) + ' seconds.')
        rospy.sleep(userdata.duration)
        return 'done'
    
class PickChecker(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success',
                                       'error'],
                             input_keys=['desired_gripper_position'])
        self._desired_gripper_position = 0.0
        self._keep_going = True
        self._success = False
        
    def jointStateCB(self, data):
        if self._keep_going:
            for joint in data.name:
                if joint == "gripper":
                    joint_nr = data.name.index(joint)
                    if (data.position[joint_nr] >= (self._desired_gripper_position + 0.05)):
                        rospy.loginfo("Gripper is at the desired state! (current: " + str(data.position[joint_nr])
                                     + ", desired: " + str(self._desired_gripper_position) + ")")
                        self._success = True
                        self._keep_going =False
                    else:
                        rospy.logerr("Gripper is not in the desired state! (current: " + str(data.position[joint_nr])
                                     + ", desired: " + str(self._desired_gripper_position) + ")")
                        self._success = False
                        self._keep_going =False
                    return
            rospy.logerr("Couldn't find a joint state for 'gripper'")
            self._success = False
            self._keep_going =False
        return
                
    def execute(self, userdata):
        self._desired_gripper_position = userdata.desired_gripper_position
        self._sub_joint_states = rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.jointStateCB)
        
        while not (rospy.is_shutdown() or not(self._keep_going)):
            rospy.loginfo("Waiting for incoming joint state message ...")
            rospy.sleep(0.5)
        if self._success:
            return 'success'
        else:
            return 'error'

@smach.cb_interface(input_keys=['motors'])
class EnableMotors(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=['motors'])
        self._pub_motor_power = rospy.Publisher("enable", std_msgs.msg.String, latch = True)
        
    def execute(self, userdata):
        msg = std_msgs.msg.String()
        for motor in userdata.motors:
            msg.data = str(motor)
            rospy.loginfo("Enabling motor '" + str(msg.data) + "'.")
            self._pub_motor_power.publish(msg)
            rospy.sleep(0.3)
        return 'success'

class GetRobotState(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success'],
                             input_keys=['robot_state'],
                             output_keys=['robot_state'])
        self._robot_state = moveit_msgs.msg.RobotState
        self._keep_listening = True
        
    def getRobotStateCB(self, msg):
        if self._keep_listening:
            self._robot_state = msg.robot_state
            self._keep_listening = False
        return
        
    def execute(self, userdata):
        rospy.loginfo("Setting up subscriber on topic " + rospy.resolve_name("move_group/monitored_planning_scene"))
        self._sub_joint_states = rospy.Subscriber("move_group/monitored_planning_scene",
                                                  moveit_msgs.msg.PlanningScene,
                                                  self.getRobotStateCB)
        
        while not (rospy.is_shutdown() or not(self._keep_listening)):
            rospy.loginfo("Waiting for incoming robot state message ...")
            rospy.sleep(0.5)
        userdata.robot_state = self._robot_state
        return 'success'

class MoveItErrorCodesParser(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success',
                                       'parsed',
                                       'no_ik_solution'],
                             input_keys=['error_code'],
                             output_keys=['result'])
        self.error_code_dict = {moveit_msgs.msg.MoveItErrorCodes.SUCCESS:'SUCCESS',
                                moveit_msgs.msg.MoveItErrorCodes.FAILURE:'FAILURE',
                                moveit_msgs.msg.MoveItErrorCodes.INVALID_MOTION_PLAN:'INVALID_MOTION_PLAN',
                                moveit_msgs.msg.MoveItErrorCodes.PLANNING_FAILED:'PLANNING_FAILED',
                                moveit_msgs.msg.MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                                moveit_msgs.msg.MoveItErrorCodes.CONTROL_FAILED:'CONTROL_FAILED',
                                moveit_msgs.msg.MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:'UNABLE_TO_AQUIRE_SENSOR_DATA',
                                moveit_msgs.msg.MoveItErrorCodes.TIMED_OUT:'TIMED_OUT',
                                moveit_msgs.msg.MoveItErrorCodes.PREEMPTED:'PREEMPTED',
                                moveit_msgs.msg.MoveItErrorCodes.START_STATE_IN_COLLISION:'START_STATE_IN_COLLISION',
                                moveit_msgs.msg.MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:'START_STATE_VIOLATES_PATH_CONSTRAINTS',
                                moveit_msgs.msg.MoveItErrorCodes.GOAL_IN_COLLISION:'GOAL_IN_COLLISION',
                                moveit_msgs.msg.MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:'GOAL_VIOLATES_PATH_CONSTRAINTS',
                                moveit_msgs.msg.MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:'GOAL_CONSTRAINTS_VIOLATED',
                                moveit_msgs.msg.MoveItErrorCodes.INVALID_LINK_NAME:'INVALID_GROUP_NAME',
                                moveit_msgs.msg.MoveItErrorCodes.INVALID_LINK_NAME:'INVALID_GOAL_CONSTRAINTS',
                                moveit_msgs.msg.MoveItErrorCodes.INVALID_LINK_NAME:'INVALID_ROBOT_STATE',
                                moveit_msgs.msg.MoveItErrorCodes.INVALID_LINK_NAME:'INVALID_LINK_NAME',
                                moveit_msgs.msg.MoveItErrorCodes.INVALID_LINK_NAME:'INVALID_OBJECT_NAME',
                                moveit_msgs.msg.MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:'FRAME_TRANSFORM_FAILURE',
                                moveit_msgs.msg.MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:'COLLISION_CHECKING_UNAVAILABLE',
                                moveit_msgs.msg.MoveItErrorCodes.ROBOT_STATE_STALE:'ROBOT_STATE_STALE',
                                moveit_msgs.msg.MoveItErrorCodes.SENSOR_INFO_STALE:'SENSOR_INFO_STALE',
                                moveit_msgs.msg.MoveItErrorCodes.NO_IK_SOLUTION:'NO_IK_SOLUTION'}
        
    def execute(self, userdata):
        result = pick_and_place_msgs.msg.MoveArmResult()
        result.error_code = userdata.error_code
        if userdata.error_code in self.error_code_dict:
            result.error_message = self.error_code_dict[userdata.error_code]
            userdata.result = result
            rospy.loginfo("Move arm finished with error message: " + self.error_code_dict[userdata.error_code])
            if userdata.error_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                return 'success'
            elif userdata.error_code == moveit_msgs.msg.MoveItErrorCodes.NO_IK_SOLUTION:
                return 'no_ik_solution'
        else:
            result.error_message = str("Error code '" + str(userdata.error_code) + "' not in dictionary. " 
                                       +"Check MoveItErrorCodes message for more information!")
            userdata.result = result
            rospy.loginfo("Error code '" + str(userdata.error_code) + "' not in dictionary. " 
                          +"Check MoveItErrorCodes message for more information!")
        return 'parsed'
