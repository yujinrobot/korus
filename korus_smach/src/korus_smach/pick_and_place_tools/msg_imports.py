#!/usr/bin/env python

# ROS
import roslib; roslib.load_manifest('korus_smach')
import tf
#from tf import TransformListener
#from tf.transformations import quaternion_from_euler

# ROS msgs
import std_msgs
import std_srvs
from std_srvs.srv import Empty
import sensor_msgs
import control_msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#import tabletop_object_detector
#from tabletop_object_detector.srv import TabletopDetection
#from tabletop_object_detector.srv import TabletopDetectionRequest
#from tabletop_object_detector.srv import TabletopDetectionResponse
#from tabletop_object_detector.msg import TabletopDetectionResult
import geometry_msgs
#from geometry_msgs.msg import TransformStamped
#from geometry_msgs.msg import Pose
#from geometry_msgs.msg import Quaternion
import trajectory_msgs
#from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_msgs
from moveit_msgs.msg import MoveGroupAction
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import shape_msgs

# Korus
import korus_smach
import pick_and_place_msgs
from pick_and_place_msgs.msg import MoveArmAction
from pick_and_place_msgs.msg import MoveArmGoal
from pick_and_place_msgs.msg import MoveArmFeedback
from pick_and_place_msgs.msg import MoveArmResult