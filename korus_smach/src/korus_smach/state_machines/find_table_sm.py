#!/usr/bin/env python
import tf
from tf import TransformListener
from state_machines_imports import *
from korus_smach.pick_and_place_tools import object_recognition, trajectory_control, misc_tools
from korus_smach.pick_and_place_tools.msg_imports import *


class CheckTablePose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['look_around',
                                       'is_zero',
                                       'is_not_zero'],
                             input_keys=['look_around',
                                         'table_pose'],
                             output_keys=[])
    
    def execute(self, userdata):
        if userdata.look_around:
            return 'look_around'
        else:
            if (userdata.table_pose.pose.position.x == 0.0 and
                userdata.table_pose.pose.position.y == 0.0 and
                userdata.table_pose.pose.position.z == 0.0):
                return 'is_zero'
            else:
                return 'is_not_zero'

class TryAgain(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['bottom_centre',
                                       'bottom_right',
                                       'top_left',
                                       'top_centre',
                                       'top_right',
                                       'done'],
                             input_keys=['reset',
                                         'look_around'])
        self._state = 0
    def execute(self, userdata):
        if userdata.reset:
            self._state = 0
            rospy.loginfo("TryAgain has been reseted.")
            return 'done'
        if userdata.look_around:
            if self._state == 0:
                self._state = 1
                return 'bottom_centre'
            elif self._state == 1:
                self._state = 2
                return 'bottom_right'
            elif self._state == 2:
                self._state = 3
                return 'top_right'
            elif self._state == 3:
                self._state = 4
                return 'top_centre'
            elif self._state == 4:
                self._state = 5
                return 'top_left'
            elif self._state == 5:
                self._state = 5
                rospy.loginfo("Whole field of view has been checked.")
                return 'done'
        else:
            rospy.logdebug("Looking around not requested. Done.")
            return 'done'

def createSM():
    sm_find_table = smach.StateMachine(outcomes=['table_found',
                                                 'no_table_found',
                                                 'preempted',
                                                 'aborted'],
                                        input_keys=['table_pose',
                                                    'look_around',
                                                    'tabletop_centre_pose',
                                                    'tf_listener'],
                                        output_keys=['tabletop_centre_pose',
                                                     'error_message',
                                                     'error_code',
                                                     'tf_listener'])
    with sm_find_table:
        # move head
        sm_find_table.userdata.motors = ['head_pan', 'head_tilt']
        sm_find_table.userdata.pose_bottom_left = geometry_msgs.PoseStamped()
        sm_find_table.userdata.pose_bottom_left.header.stamp = rospy.Time.now()
        sm_find_table.userdata.pose_bottom_left.header.frame_id = "base_footprint"
        sm_find_table.userdata.pose_bottom_left.pose.position.x = 0.2
        sm_find_table.userdata.pose_bottom_left.pose.position.y = -0.2
        sm_find_table.userdata.pose_bottom_left.pose.position.z = 0.0
        sm_find_table.userdata.pose_bottom_left.pose.orientation.w = 1.0
        sm_find_table.userdata.pose_bottom_centre = geometry_msgs.PoseStamped()
        sm_find_table.userdata.pose_bottom_centre.header = sm_find_table.userdata.pose_bottom_left.header
        sm_find_table.userdata.pose_bottom_centre.pose.position.x = 0.2
        sm_find_table.userdata.pose_bottom_centre.pose.position.y = 0.0
        sm_find_table.userdata.pose_bottom_centre.pose.position.z = 0.0
        sm_find_table.userdata.pose_bottom_centre.pose.orientation.w = 1.0
        sm_find_table.userdata.pose_bottom_right = geometry_msgs.PoseStamped()
        sm_find_table.userdata.pose_bottom_right.header = sm_find_table.userdata.pose_bottom_left.header
        sm_find_table.userdata.pose_bottom_right.pose.position.x = 0.2
        sm_find_table.userdata.pose_bottom_right.pose.position.y = 0.2
        sm_find_table.userdata.pose_bottom_right.pose.position.z = 0.0
        sm_find_table.userdata.pose_bottom_right.pose.orientation.w = 1.0
        sm_find_table.userdata.pose_top_left = geometry_msgs.PoseStamped()
        sm_find_table.userdata.pose_top_left.header = sm_find_table.userdata.pose_bottom_left.header
        sm_find_table.userdata.pose_top_left.pose.position.x = 0.5
        sm_find_table.userdata.pose_top_left.pose.position.y = -0.5
        sm_find_table.userdata.pose_top_left.pose.position.z = 0.5
        sm_find_table.userdata.pose_top_left.pose.orientation.w = 1.0
        sm_find_table.userdata.pose_top_centre = geometry_msgs.PoseStamped()
        sm_find_table.userdata.pose_top_centre.header = sm_find_table.userdata.pose_bottom_left.header
        sm_find_table.userdata.pose_top_centre.pose.position.x = 0.5
        sm_find_table.userdata.pose_top_centre.pose.position.y = 0.0
        sm_find_table.userdata.pose_top_centre.pose.position.z = 0.5
        sm_find_table.userdata.pose_top_centre.pose.orientation.w = 1.0
        sm_find_table.userdata.pose_top_right = geometry_msgs.PoseStamped()
        sm_find_table.userdata.pose_top_right.header = sm_find_table.userdata.pose_bottom_left.header
        sm_find_table.userdata.pose_top_right.pose.position.x = 0.5
        sm_find_table.userdata.pose_top_right.pose.position.y = 0.5
        sm_find_table.userdata.pose_top_right.pose.position.z = 0.5
        sm_find_table.userdata.pose_top_right.pose.orientation.w = 1.0
        sm_find_table.userdata.pose_centre = geometry_msgs.PoseStamped()
        sm_find_table.userdata.pose_centre.header = sm_find_table.userdata.pose_bottom_left.header
        sm_find_table.userdata.pose_centre.pose.position.x = 0.5
        sm_find_table.userdata.pose_centre.pose.position.y = 0.0
        sm_find_table.userdata.pose_centre.pose.position.z = 0.0
        sm_find_table.userdata.pose_centre.pose.orientation.w = 1.0
        # object recognition
        sm_find_table.userdata.error_message = str()
        sm_find_table.userdata.error_code = int()
        # wait
        sm_find_table.userdata.wait_0sec = 0.0
        sm_find_table.userdata.wait_1sec = 1.0
        sm_find_table.userdata.wait_3sec = 3.0
        sm_find_table.userdata.wait_5sec = 5.0
        sm_find_table.userdata.wait_10sec = 10.0
        # reset TryAgain
        sm_find_table.userdata.reset_true = True
        sm_find_table.userdata.reset_false = False
        # reset TryAgain
        sm_find_table.userdata.reset_true = True
        sm_find_table.userdata.reset_false = False
        sm_find_table.userdata.objects_info = []
        # 3D sensor modes
        sm_find_table.userdata.color_mode_high_res = 5
        sm_find_table.userdata.depth_mode_high_res = sm_find_table.userdata.color_mode_high_res
        sm_find_table.userdata.ir_mode_high_res = sm_find_table.userdata.color_mode_high_res
        sm_find_table.userdata.color_mode_low_res = 8
        sm_find_table.userdata.depth_mode_low_res = sm_find_table.userdata.color_mode_low_res
        sm_find_table.userdata.ir_mode_low_res = sm_find_table.userdata.color_mode_low_res
        
        smach.StateMachine.add('EnableHighResPointCloud',
                               misc_tools.change3DSensorDriverMode(),
                               transitions={'done':'EnableMotors'},
                               remapping={'color_mode':'color_mode_high_res',
                                          'depth_mode':'depth_mode_high_res',
                                          'ir_mode':'ir_mode_high_res'})
        
        smach.StateMachine.add('EnableMotors',
                               misc_tools.EnableMotors(),
                               transitions={'success':'ResetTryAgain'},
                               remapping={'motors':'motors'})

        try_again_sm = TryAgain()
        smach.StateMachine.add('ResetTryAgain',
                               try_again_sm,
                               remapping={'reset':'reset_true',
                                          'look_around':'look_around'},
                               transitions={'bottom_centre':'aborted',
                                            'bottom_right':'aborted',
                                            'top_right':'aborted',
                                            'top_centre':'aborted',
                                            'top_left':'aborted',
                                            'done':'CheckTablePose'})
        
        smach.StateMachine.add('CheckTablePose',
                               CheckTablePose(),
                               transitions={'look_around':'MoveHeadBottomLeft',
                                            'is_zero':'RecognizeObjects',
                                            'is_not_zero':'MoveHead'},
                               remapping={'look_around':'look_around',
                                          'table_pose':'table_pose'})
        
        smach.StateMachine.add('MoveHead',
                               SimpleActionState('head_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.headControlRequestCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['tf_listener',
                                                             'goal_pose'],
                                                 output_keys=['tf_listener',
                                                              'error_code']),
                               remapping={'tf_listener':'tf_listener',
                                          'goal_pose':'table_pose',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveHeadBottomLeft',
                               SimpleActionState('head_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.headControlRequestCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['tf_listener',
                                                             ' goal_pose'],
                                                 output_keys=['tf_listener',
                                                              'error_code']),
                               remapping={'tf_listener':'tf_listener',
                                          'goal_pose':'pose_bottom_left',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveHeadBottomCentre',
                               SimpleActionState('head_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.headControlRequestCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['tf_listener',
                                                             'goal_pose'],
                                                 output_keys=['tf_listener',
                                                              'error_code']),
                               remapping={'tf_listener':'tf_listener',
                                          'goal_pose':'pose_bottom_centre',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveHeadBottomRight',
                               SimpleActionState('head_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.headControlRequestCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['tf_listener',
                                                             'goal_pose'],
                                                 output_keys=['tf_listener',
                                                              'error_code']),
                               remapping={'tf_listener':'tf_listener',
                                          'goal_pose':'pose_bottom_right',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveHeadTopRight',
                               SimpleActionState('head_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.headControlRequestCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['tf_listener',
                                                             'goal_pose'],
                                                 output_keys=['error_code']),
                               remapping={'tf_listener':'tf_listener',
                                          'goal_pose':'pose_top_right',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveHeadTopCentre',
                               SimpleActionState('head_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.headControlRequestCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['tf_listener',
                                                             'goal_pose'],
                                                 output_keys=['error_code']),
                               remapping={'tf_listener':'tf_listener',
                                          'goal_pose':'pose_top_centre',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('MoveHeadTopLeft',
                               SimpleActionState('head_controller',
                                                 control_msgs.FollowJointTrajectoryAction,
                                                 goal_cb=trajectory_control.headControlRequestCb,
                                                 result_cb=trajectory_control.generalResponseCb,
                                                 input_keys=['tf_listener',
                                                             'goal_pose'],
                                                 output_keys=['error_code']),
                               remapping={'tf_listener':'tf_listener',
                                          'goal_pose':'pose_top_left',
                                          'error_code':'error_code'},
                               transitions={'succeeded':'ParseMoveHeadErrorCode',
                                            'aborted':'ParseMoveHeadErrorCode',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('ParseMoveHeadErrorCode',
                               trajectory_control.FollowJointTrajectoryErrorCodesParser(),
                               transitions={'success':'WaitForPointcloudAlignment',
                                            'parsed':'aborted'},
                               remapping={'error_code':'error_code',
                                          'error_message':'error_message'})
        
        smach.StateMachine.add('WaitForPointcloudAlignment',
                               misc_tools.Wait(),
                               remapping={'duration':'wait_1sec'},
                               transitions={'done':'RecognizeObjects'})
         
        smach.StateMachine.add('RecognizeObjects',
                               SimpleActionState('recognize_objects',
                                                 object_recognition_msgs.ObjectRecognitionAction,
                                                 input_keys=[],
                                                 output_keys=[]),
                                remapping={},
                                transitions={'succeeded':'AddTableToPlanningScene',
                                             'preempted':'preempted',
                                             'aborted':'aborted'})
        
        smach.StateMachine.add('AddTableToPlanningScene',
                               object_recognition.GetTable(),
                               remapping={'tabletop_centre_pose':'tabletop_centre_pose',
                                          'tf_listener':'tf_listener'},
                               transitions={'table_added':'DisableHighResPointCloudSuccess',
                                            'no_table_added':'DisableHighResPointCloudNoSuccess'})
        
        smach.StateMachine.add('TryAgain',
                               try_again_sm,
                               remapping={'reset':'reset_false',
                                          'look_around':'look_around'},
                               transitions={'bottom_centre':'MoveHeadBottomCentre',
                                            'bottom_right':'MoveHeadBottomRight',
                                            'top_right':'MoveHeadTopRight',
                                            'top_centre':'MoveHeadTopCentre',
                                            'top_left':'MoveHeadTopLeft',
                                            'done':'DisableHighResPointCloudNoSuccess'})
        
        smach.StateMachine.add('DisableHighResPointCloudNoSuccess',
                               misc_tools.change3DSensorDriverMode(),
                               transitions={'done':'no_table_found'},
                               remapping={'color_mode':'color_mode_low_res',
                                          'depth_mode':'depth_mode_low_res',
                                          'ir_mode':'ir_mode_low_res'})
        
        smach.StateMachine.add('DisableHighResPointCloudSuccess',
                               misc_tools.change3DSensorDriverMode(),
                               transitions={'done':'table_found'},
                               remapping={'color_mode':'color_mode_low_res',
                                          'depth_mode':'depth_mode_low_res',
                                          'ir_mode':'ir_mode_low_res'})
        
    return sm_find_table