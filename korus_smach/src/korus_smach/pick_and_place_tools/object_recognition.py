#!/usr/bin/env python
import tf
from korus_smach.state_machines.state_machines_imports import *
from korus_smach.pick_and_place_tools.msg_imports import *

#===============================================================================================================
# Detect object callbacks
#===============================================================================================================
def objectRecognitionGoalCb(userdata, goal):
    goal = object_recognition_msgs.ObjectRecognitionGoal()
    return goal

@smach.cb_interface(input_keys=['min_confidence',
                                'recognised_objects',
                                 'object_names',
                                'error_code',
                                'error_message',
                                'tf_listener'],
                    output_keys=['recognised_objects',
                                 'object_names',
                                 'error_code',
                                 'error_message',
                                 'tf_listener'],
                    outcomes=['succeeded',
                              'no_objects_found',
                              'preempted',
                              'aborted'])
def objectRecognitionResultCb(userdata, status, result):
    if status == actionlib_msgs.GoalStatus.SUCCEEDED:
        reduced_model_set_dict = {'18665':'000.580.67',
                                  '18685':'501.245.12',
                                  '18691':'800.572.57',
                                  '18693':'801.327.80',
                                  '18699':'901.334.73',
                                  '18744':'coke_classic',
                                  '18746':'mach3_gel',
                                  '18765':'coffee-mate',
                                  '18766':'suave-kids-3in1',
                                  '18783':'izze_can',
                                  '18791':'v8_bottle',
                                  '18798':'tennis_ball_can',
                                  '18799':'clearasil_jar',
                                  '18800':'contact_lens_cleaner',
                                  '18802':'hydrogen_peroxide_bottle',
                                  '18807':'campbell_soup_can',
                                  '18808':'campbell_soup_handheld'}
        rospy.logdebug("Object recognition was successful. Processing the result ...")
        userdata.recognised_objects = object_recognition_msgs.RecognizedObjectArray()
        userdata.object_names = list()
        unreliable_recognition = int()
        for object in result.recognized_objects.objects:
            if object.confidence >= userdata.min_confidence:
                if object.type.key in reduced_model_set_dict:
                    userdata.object_names.append(reduced_model_set_dict[object.type.key])
                    rospy.loginfo("Added recognised object '" + reduced_model_set_dict[object.type.key] + "' (" 
                                   + object.type.key + ") with confidence " 
                                   + str(object.confidence))
                    ''' convert poses to robot_root since 3d sensor pose keeps changing '''
                    try:
                        print"pose before"
                        print object.pose
                        new_pose = geometry_msgs.PoseStamped()
                        new_pose.header = object.pose.header
                        new_pose.pose = object.pose.pose.pose
                        new_pose.header.stamp = userdata.tf_listener.getLatestCommonTime(
                                                               "base_footprint", object.pose.header.frame_id)
                        new_pose = userdata.tf_listener.transformPose("base_footprint", new_pose)
                        object.pose.header = new_pose.header
                        object.pose.pose.pose = new_pose.pose
                        print"pose after"
                        print object.pose
                    except tf.Exception, e:
                        rospy.logerr('Couldn`t transform requested pose!')
                        rospy.logerr('%s', e)
                    userdata.recognised_objects.objects.append(object)
                else:
                    userdata.error_message = "Object ID '" + str(object.type.key) + "' not in dictionary!"
                    rospy.logerr(userdata.error_message)
                    return 'aborted'
            else:
                unreliable_recognition += 1
                rospy.loginfo("Recognised object '" + reduced_model_set_dict[object.type.key] + "' (" 
                                   + object.type.key + "), but confidence is too low (" 
                                   + str(reduced_model_set_dict[object.type.key]) + ").")
        if len(result.recognized_objects.objects) == 0:
            userdata.error_message = "No objects found"
            rospy.loginfo(userdata.error_message)
            return 'no_objects_found'
        elif unreliable_recognition == len(result.recognized_objects.objects):
            userdata.error_message = "Objects found, but recognition was not reliable."
            rospy.loginfo(userdata.error_message)
            return 'no_objects_found'
        else:
            userdata.error_message = "Object recognition succeeded."
            rospy.loginfo(userdata.error_message)
            return 'succeeded'
    elif status == actionlib_msgs.GoalStatus.PREEMPTED:
        userdata.error_message = "Object recognition was preempted."
        rospy.logwarn(userdata.error_message)
        return 'preempted'
    else:
        userdata.error_message = "Object recognition failed!"
        rospy.logerr(userdata.error_message)
        return 'aborted'

class GetObjectInformation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['done'],
                             input_keys=['recognised_objects',
                                         'error_code',
                                         'error_message'],
                             output_keys=['objects_info',
                                          'error_code',
                                          'error_message'])
    def execute(self, userdata):
        rospy.loginfo("Waiting for 'get_object_info' service ... ")
        rospy.wait_for_service("get_object_info")
        rospy.loginfo("'get_object_info' service available.")
        objects_info = list()
        srv_client = rospy.ServiceProxy("get_object_info",
                                         object_recognition_srvs.GetObjectInformation())
        for object in userdata.recognised_objects.objects:
            try:
                info_request = object_recognition_srvs.GetObjectInformationRequest()
                info_request.type = object.type
                objects_info.append(srv_client(info_request))
            except rospy.ServiceException, e:
                rospy.loginfo("Service did not process request: " + str(e))
        userdata.objects_info = objects_info
        return 'done'
