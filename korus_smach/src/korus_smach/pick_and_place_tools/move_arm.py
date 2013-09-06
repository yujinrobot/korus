#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('korus_smach')
from korus_smach.state_machines.state_machines_imports import *
from korus_smach.pick_and_place_tools.msg_imports import *

#======================================================================================================================
# Move Arm SM goal and result callbacks 
#======================================================================================================================
@smach.cb_interface(input_keys=['goal_pose',
                                'goal_link_name',
                                'robot_state'])
def moveArmGoalCB(userdata, goal):
    goal_constraint = moveit_msgs.msg.Constraints()
    goal_constraint.name = "goal"
    # Define position constraint
    position_constraint = moveit_msgs.msg.PositionConstraint()
    position_constraint.header = userdata.goal_pose.header
    position_constraint.link_name = userdata.goal_link_name
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0
    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = shape_msgs.msg.SolidPrimitive.SPHERE
#    primitive.dimensions.append(1e-5)
    primitive.dimensions.append(0.0)
    position_constraint.constraint_region.primitives.append(primitive)
    primitive_pose = geometry_msgs.msg.Pose()
    # Adapt position to Korus' special kinematics
    angle = math.atan2(userdata.goal_pose.pose.position.y, userdata.goal_pose.pose.position.x)
    dist = math.sqrt(math.pow(userdata.goal_pose.pose.position.x, 2) + math.pow(userdata.goal_pose.pose.position.y, 2))
    primitive_pose.position.x = dist * math.cos(angle)
    primitive_pose.position.y = dist * math.sin(angle)
    primitive_pose.position.z = userdata.goal_pose.pose.position.z
    primitive_pose.orientation.w = 1.0
    position_constraint.constraint_region.primitive_poses.append(primitive_pose)
    position_constraint.weight = 1.0
    goal_constraint.position_constraints.append(position_constraint)
    # Define orientation constraint
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.header = userdata.goal_pose.header
    orientation_constraint.link_name = position_constraint.link_name
    # Adapt orientation to Korus' special kinematics
    yaw = angle
    roll = math.pi / 2
    pitch = 0.0
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation_constraint.orientation = geometry_msgs.msg.Quaternion(*quat)
    orientation_constraint.absolute_x_axis_tolerance = 1e-5 #0.00017453293 # # 1 deg, optional
    orientation_constraint.absolute_y_axis_tolerance = 1e-5 #0.00017453293 # optional
    orientation_constraint.absolute_z_axis_tolerance = 1e-5 #0.00017453293 # optional
    orientation_constraint.weight = 1.0
    goal_constraint.orientation_constraints.append(orientation_constraint)
    rospy.loginfo("Goal constraints: ")
    rospy.loginfo(position_constraint)
    rospy.loginfo(orientation_constraint)
    
    motion_plan_request = moveit_msgs.msg.MotionPlanRequest()
    ws_params = moveit_msgs.msg.WorkspaceParameters()
    ws_params.header = userdata.goal_pose.header
    ws_params.min_corner.x = 0.0
    ws_params.max_corner.x = 1.0
    ws_params.min_corner.y = -1.0
    ws_params.max_corner.y = 1.0
    ws_params.min_corner.z = 0.0
    ws_params.max_corner.z = 1.5
    motion_plan_request.workspace_parameters = ws_params
    #motion_plan_request.start_state = userdata.robot_state # not needed
    motion_plan_request.goal_constraints.append(goal_constraint)
    motion_plan_request.path_constraints = moveit_msgs.msg.Constraints()
    motion_plan_request.trajectory_constraints = moveit_msgs.msg.TrajectoryConstraints()
    motion_plan_request.planner_id = "BKPIECEkConfigDefault" # using default needed
    motion_plan_request.group_name = "arm"
    motion_plan_request.num_planning_attempts = 1
    motion_plan_request.allowed_planning_time = 10.0
    move_arm_goal = moveit_msgs.msg.MoveGroupGoal()
    move_arm_goal.request = motion_plan_request
    move_arm_goal.planning_options = moveit_msgs.msg.PlanningOptions()
    rospy.loginfo('Move arm goal prepared.')
    return move_arm_goal

@smach.cb_interface(input_keys=['robot_goal_state',
                                'arm_joint_names'])
def moveArmJointGoalCB(userdata, goal):
    goal_constraint = moveit_msgs.msg.Constraints()
    goal_constraint.name = "joint_goal"
    # Define joint constraints
    for name in userdata.robot_goal_state.joint_state.name:
        if name in userdata.arm_joint_names:
            joint_constraint = moveit_msgs.msg.JointConstraint()
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            joint_constraint.joint_name = name
            joint_index = userdata.robot_goal_state.joint_state.name.index(name)
            joint_constraint.position = userdata.robot_goal_state.joint_state.position[joint_index]
            goal_constraint.joint_constraints.append(joint_constraint)
#    rospy.loginfo("Goal constraint: ")
#    rospy.loginfo(goal_constraint)
    motion_plan_request = moveit_msgs.msg.MotionPlanRequest()
    ws_params = moveit_msgs.msg.WorkspaceParameters()
    ws_params.header = userdata.robot_goal_state.joint_state.header
    ws_params.min_corner.x = -1.0
    ws_params.min_corner.y = -1.0
    ws_params.min_corner.z = 0.0
    ws_params.max_corner.x = 1.0
    ws_params.max_corner.y = 1.0
    ws_params.max_corner.z = 1.5
    motion_plan_request.workspace_parameters = ws_params
#    motion_plan_request.start_state = userdata.robot_state # not needed
    motion_plan_request.goal_constraints.append(goal_constraint)
    motion_plan_request.path_constraints = moveit_msgs.msg.Constraints()
    motion_plan_request.trajectory_constraints = moveit_msgs.msg.TrajectoryConstraints()
    motion_plan_request.planner_id = "LBKPIECEkConfigDefault" # using default needed
    motion_plan_request.group_name = "arm"
    motion_plan_request.num_planning_attempts = 1
    motion_plan_request.allowed_planning_time = 10.0
    move_arm_goal = moveit_msgs.msg.MoveGroupGoal()
    move_arm_goal.request = motion_plan_request
    move_arm_goal.planning_options = moveit_msgs.msg.PlanningOptions()
    rospy.loginfo('Move arm goal prepared.')
    return move_arm_goal

@smach.cb_interface(input_keys=['error_code'],
                    output_keys=['error_code'],
                    outcomes=['succeeded',
                              'aborted'])
def moveArmResultCB(userdata, status, result):
    userdata.error_code = result.error_code.val
    if result.error_code.val is moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        return 'succeeded'
    else:
        return 'aborted'