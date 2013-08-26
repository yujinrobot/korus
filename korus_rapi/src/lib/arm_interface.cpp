/**
 * @file /qkorus/src/arm_interface.cpp
 *
 * @brief Implementation of the arm connections.
 *
 * @date October 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <manipulation_comms/MoveArmGoal.h>
#include <std_msgs/String.h>
#include "../../include/korus_rapi/arm_interface.hpp"
//#include <kinematics_msgs/GetPositionFK.h>
//#include <kinematics_msgs/GetPositionIK.h>
#include <device_msgs/StringJointParameters.h>


#include <geometry_msgs/PoseStamped.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace korus_rapi {

/*****************************************************************************
** Implementation
*****************************************************************************/

ArmInterface::ArmInterface() :
		arm_controller_state_topic("/korus/arm_controller/state"),
		enable_topic("/korus/enable"),
		disable_topic("/korus/disable"),
		get_fk_topic("/move_arm/arm_fk"),
		get_ik_topic("/move_arm/arm_ik"),
		get_fk_solver_info_topic("/move_arm/arm_fk_solver_info"),
		get_ik_solver_info_topic("/move_arm/arm_ik_solver_info"),
		action_client("/korus/arm_controller"),
//		action_client1("/korus/move_arm_ik"),
		move_arm_connected(false),
		inverse_kinematics_connected(false),
		sig_response("sig_arm_command_interface")
{
	/*********************
	** Ros
	**********************/
	ros::NodeHandle nh("~");
	enable_publisher = nh.advertise<std_msgs::String>(enable_topic, 100);
	disable_publisher = nh.advertise<std_msgs::String>(disable_topic, 100);
	arm_controller_state_subscriber = nh.subscribe(arm_controller_state_topic,10,&ArmInterface::subscribeArmControllerState, this);

	/*********************
	** Names
	**********************/
	joint_names.push_back("torso_turn");
	joint_names.push_back("torso_lift");
	joint_names.push_back("shoulder");
	joint_names.push_back("elbow");
	joint_names.push_back("wrist");

	/*********************
	** Names
	**********************/
//	ros::ServiceClient mechanism_client = nh.serviceClient<device_msgs::StringJointParameters>("/korus/mechanism_model/joint_parameters");
//    device_msgs::StringJointParameters srv;
//    for ( unsigned int i = 0; i < joint_names.size(); ++i ) {
//        srv.request.name = joint_names[i];
//    	if ( !mechanism_client.call(srv)) {
//    		ROS_WARN_STREAM("Arm Node: failed to contact the mechanism model for joint configurations.");
//    	} else {
//    		joint_configuration.push_back(srv.response.parameters);
//    	}
//    }

	/*********************
	** Parameters Model
	**********************/
	if ( !action_client.waitForServer( ros::Duration(3.0)) ) {
		ROS_WARN_STREAM("Could not connect to the move_arm action server1.");
	}

	/*********************
	** Joint Model
	**********************/
	if ( !action_client.waitForServer( ros::Duration(3.0)) ) {
		ROS_WARN_STREAM("Could not connect to the move_arm action server2.");
	}


//	if ( !action_client1.waitForServer( ros::Duration(3.0)) ) {
//		ROS_WARN_STREAM("Could not connect to the move_arm action server3.");
//	}

	/*********************
	** Joint Model
	**********************/
//	if ( !action_client1.waitForServer( ros::Duration(3.0)) ) {
//		ROS_WARN_STREAM("Could not connect to the move_arm action server4.");
//	}
	/*********************
	** Qt Model - for work space
	**********************/
//	// wait for service
//	inverse_kinematics_connected = true;
//	if ( !ros::service::waitForService(get_fk_topic, ros::Duration(1)) ) {
//		ROS_WARN_STREAM("Could not connect to the inverse kinematics server.");
//		inverse_kinematics_connected = false;
//	}
//	if ( !ros::service::waitForService(get_fk_topic, ros::Duration(1)) ) {
//		ROS_WARN_STREAM("Could not connect to the inverse kinematics server.");
//		inverse_kinematics_connected = false;
//	}


//	// I'd like to display solver information
//	kinematics_msgs::GetKinematicSolverInfo::Request request;
//	std::cout << "Request fk solver information" << std::endl;
//	if (ros::service::call(get_fk_solver_info_topic, request, response) )
//	{
//		for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//		{
//			std::cout << "   joint: "  << response.kinematic_solver_info.joint_names[i] << std::endl;
//		}
//
//		for(unsigned int i=0; i< response.kinematic_solver_info.link_names.size(); i++)
//		{
//			std::cout << "   link: " << response.kinematic_solver_info.link_names[i] << std::endl;
//		}
//	}
//	else
//	{
//		std::cout << "   could not got fk solver information; it may not work!" << std::endl;
//	}
//
//	// I'd like to know information about ik solver
//	std::cout << "Request ik solver information" << std::endl;
//	 if (ros::service::call(get_ik_solver_info_topic, request, response) )
//	{
//		 for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//		{
//			std::cout << "   joint: " << response.kinematic_solver_info.joint_names[i] << std::endl;
//			ik_result.push_back( 0.0 );
//		}
//
//		for(unsigned int i=0; i< response.kinematic_solver_info.link_names.size(); i++)
//		{
//			std::cout << "   link: " << response.kinematic_solver_info.link_names[i] << std::endl;
//		}
//	}
//	else
//	{
//		std::cout << "   could not got ik solver information; it may not work!" << std::endl;
//	}

}


ArmInterface::~ArmInterface() {
	arm_controller_state_subscriber.shutdown(); // make sure we dont' do callbacks after the gui shut down.
}

/*****************************************************************************
** Ros
*****************************************************************************/

/**
 * joint state callback
 * At this, we will display joint position and end-effector pose information on mode_workspace
 * @param states
 */
void ArmInterface::subscribeArmControllerState(const controller_msgs::JointTrajectoryControllerState& state) {

//	kinematics_msgs::GetPositionFK::Request fk_request;
//	kinematics_msgs::GetPositionFK::Response fk_response;
//	// joint position display
//	for ( unsigned int i = 0; i < state->trajectory.actual_position.size(); ++i ) {
//		joint_angles[i] = state->trajectory.actual_position[i];
//		fk_request.robot_state.joint_state.position.push_back( state->actual[i] );
//		std::stringstream str_stream;
//		str_stream << (i+1);
//	}
//	fk_request.robot_state.joint_state.name = joint_names;
//
//
//	if (inverse_kinematics_connected ) {
//
//		// end-effector pose display
//		fk_request.header.frame_id = "base_link";									// solver will take into account pose of end-effector from the base link
//		fk_request.fk_link_names.push_back( "end_effector_link" );		// i am interesting in pose of end-effector
//		if (ros::service::call("/move_arm/arm_fk", fk_request, fk_response) )		// request the pose of the end-effector
//		{
//			end_effector_pose[0] = fk_response.pose_stamped[0].pose.position.x;
//			end_effector_pose[1] = fk_response.pose_stamped[0].pose.position.y;
//			end_effector_pose[2] = fk_response.pose_stamped[0].pose.position.z;
//			end_effector_pose[3] = fk_response.pose_stamped[0].pose.orientation.x;
//			end_effector_pose[4] = fk_response.pose_stamped[0].pose.orientation.y;
//			end_effector_pose[5] = fk_response.pose_stamped[0].pose.orientation.z;
//			end_effector_pose[6] = fk_response.pose_stamped[0].pose.orientation.w;
//		}
//		else
//		{
//			std::cout <<  "could not get the result of fk " << std::endl;
//		}
//	}
	for( unsigned int i = 0; i < state.trajectory.actual_position.size(); ++i )
	{
			joint_angles[i] = state.trajectory.actual_position[i];
	}
	joint_angles[5]=0.0;
	joint_angles[6]=0.0;

}

/*****************************************************************************
** Gui Callbacks
*****************************************************************************/

void ArmInterface::solveTargetPose( std::vector<double> & targetPose )
{
//	if( targetPose.size() != 7 )
//	{
//		std::cout << "targetPose is not 6dof " << std::endl;
//		sig_response.emit( -1 );
//		return;
//	}
//
//	kinematics_msgs::GetPositionIK::Request ik_request;
//	kinematics_msgs::GetPositionIK::Response ik_response;
//
//	ik_request.timeout = ros::Duration(5.0);
//	ik_request.ik_request.ik_link_name = "end_effector_link";
//	ik_request.ik_request.pose_stamped.header.frame_id = "base_link";
//
//	ik_request.ik_request.pose_stamped.pose.position.x = targetPose[0];
//	ik_request.ik_request.pose_stamped.pose.position.y = targetPose[1];
//	ik_request.ik_request.pose_stamped.pose.position.z = targetPose[2];
//
//	ik_request.ik_request.pose_stamped.pose.orientation.x = targetPose[3];
//	ik_request.ik_request.pose_stamped.pose.orientation.y = targetPose[4];
//	ik_request.ik_request.pose_stamped.pose.orientation.z = targetPose[5];
//	ik_request.ik_request.pose_stamped.pose.orientation.w = targetPose[6];
//
//	ik_request.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
//	ik_request.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
//	for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//	{
//		ik_request.ik_request.ik_seed_state.joint_state.position[i]
//           = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
//	}
//
//	// cal function
//	std::cout << "Request the ik" << std::endl;
//	std::stringstream str_stream;
//	str_stream << "ik_result: ";
//	if (ros::service::call(get_ik_topic, ik_request, ik_response) )
//	{
//		if(ik_response.error_code.val == ik_response.error_code.SUCCESS )
//		{
//			for(unsigned int i=0; i < ik_response.solution.joint_state.name.size(); i ++)
//			{
//				ik_result[i] = ik_response.solution.joint_state.position[i];
//				str_stream << ik_response.solution.joint_state.position[i] << ",";
//			}
//			std::cout << str_stream.str() << std::endl;
//			sendTrajectory( ik_result );
//		}
//		else
//		{
//			std::cout << "got the result of ik. However it is not successful" << std::endl;
//			sig_response.emit( -3 );
//		}
//	}
//	else
//	{
//		std::cout << "could not get result of ik within 5 sec. Please call Younghoon, Joo" << std::endl;
//		sig_response.emit( -4 );
//	}

//    korus_smach::MoveArmIKGoal action_goal;

	geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "base_footprint";
    goal_pose.pose.position.x = targetPose[0];
    goal_pose.pose.position.y = targetPose[1];
    goal_pose.pose.position.z = targetPose[2];
    goal_pose.pose.orientation.x = 0.707;
    goal_pose.pose.orientation.y = -0.002;
    goal_pose.pose.orientation.z = 0.002;
    goal_pose.pose.orientation.w = 0.707;

//    action_goal.goal_pose.header = ...
//    action_goal.goal_pose.pose.position.x = 1.0;

//    action_goal.goal_pose = goal_pose;

	enable();
	// why is this here? having to need sleeps everywhere is a bad habit,
	// it means your program is unstable -> should program better than this!
	ros::Duration sleeper(0.2);
	sleeper.sleep();

//	action_client1.sendGoal(action_goal,
//			//boost::bind(&ArmInterface::done1, this),
//			boost::bind(&ArmInterface::done1, this, _1, _2),
//			boost::bind(&ArmInterface::active1, this),
//			boost::bind(&ArmInterface::feedback1, this)
//	);

}


/**
 * This will be expanded more later if we need to add more than one
 * waypoint to the trajectory goal.
 */
void ArmInterface::sendTrajectory( std::vector<double> & jointAngles, unsigned int & mode )
{

	if( jointAngles.size() != joint_names.size() )
	{
		std::cout << "number of joint is not " << "cmd joint: "<< jointAngles.size()<<" joint names: "<<  joint_names.size() << std::endl;
		sig_response.emit( -5 );
		return;
	}

	enable();
	// why is this here? having to need sleeps everywhere is a bad habit,
	// it means your program is unstable -> should program better than this!
	ros::Duration sleeper(0.2);
	sleeper.sleep();

	if ( joint_names.size() != jointAngles.size() ) {
		ROS_ERROR("Korus rapi: incoming joint trajectory command malformed, size did not match #joints.");
		return;
	}


	//manipulation_comms::MoveArmGoal goal;
	control_msgs::FollowJointTrajectoryGoal goal;

	//goal.joint_trajectory.joint_names = joint_names;
	goal.trajectory.joint_names = joint_names;

//	manipulation_comms::JointTrajectoryPoint point;


	trajectory_msgs::JointTrajectoryPoint point; // way point



	switch (mode)
	{
	case 0: //normal
		for ( unsigned int joint = 0; joint < joint_names.size(); ++joint ) {
			point.positions.push_back(jointAngles[joint]);
			point.velocities.push_back(0.5);
			point.accelerations.push_back(0);
		}
		point.velocities[1]=0.02;
		goal.trajectory.points.push_back(point);
		break;

	case 1: //slow
		for ( unsigned int joint = 0; joint < joint_names.size(); ++joint ) {
			point.positions.push_back(jointAngles[joint]);
			point.velocities.push_back(0.2);
			point.accelerations.push_back(2.0);
		}
		point.velocities[1]=0.01;
		goal.trajectory.points.push_back(point);
		break;
	case 2: //fast
		for ( unsigned int joint = 0; joint < joint_names.size(); ++joint ) {
			point.positions.push_back(jointAngles[joint]);
			point.velocities.push_back(1.57);
			point.accelerations.push_back(5.0);
		}
		point.velocities[1]=0.04;
		point.accelerations[1]=0.5;

		goal.trajectory.points.push_back(point);
		break;
	default:
		break;
	}

	action_client.sendGoal(goal,
			//boost::bind(&ArmInterface::done, this),
			boost::bind(&ArmInterface::done, this, _1, _2),
			boost::bind(&ArmInterface::active, this),
			boost::bind(&ArmInterface::feedback, this)
	);
}

void ArmInterface::enable() {
	for ( unsigned int i = 0; i < joint_names.size(); ++i ) {
		std::cout << "Enable: " << joint_names[i] << std::endl;
		std_msgs::String name;
		name.data  = joint_names[i];
		enable_publisher.publish(name);
	}
}

void ArmInterface::disable() {
	for ( unsigned int i = 0; i < joint_names.size(); ++i ) {
		std::cout << "Disable: " << joint_names[i] << std::endl;
		std_msgs::String name;
		name.data  = joint_names[i];
		disable_publisher.publish(name);
	}
}

/*****************************************************************************
** Action Callbacks
*****************************************************************************/

void ArmInterface::feedback()
{
	std::cout << "Goal status moved to " << std::endl;
}

/** @brief This gets called as soon as the server moves the goal from pending to active. */
void ArmInterface::active()
{
	std::cout << "Goal acknowledged" << std::endl;
}

void ArmInterface::done(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
//	if ( result->error.value == manipulation_comms::ManipulationErrorCodes::SUCCESS )
//	{
//		ROS_DEBUG("Korus rapi: goal returned with success");
//		sig_response.emit( 0 );
//	}
//	else
//	{
//		ROS_WARN("Korus rapi: arm goal returned with error code #");
//		sig_response.emit( result->error.value );
//	}

//	int32 SUCCESSFUL=0
//	int32 INVALID_GOAL=-1
//	int32 INVALID_JOINTS=-2
//	int32 OLD_HEADER_TIMESTAMP=-3
//	int32 PATH_TOLERANCE_VIOLATED=-4
//	int32 GOAL_TOLERANCE_VIOLATED=-5
//	int32 error_code

	if ( result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL )
	{
		ROS_DEBUG("Korus rapi: goal returned with success");
		sig_response.emit( 0 );
	}
	else
	{
		ROS_WARN("Korus rapi: arm goal returned with error code #");
		sig_response.emit( result->error_code );
	}
}


/*****************************************************************************
** Action1 Callbacks
*****************************************************************************/

void ArmInterface::feedback1()
{
	std::cout << "Goal status moved to " << std::endl;
}

/** @brief This gets called as soon as the server moves the goal from pending to active. */
void ArmInterface::active1()
{
	std::cout << "Goal acknowledged" << std::endl;
}

//void ArmInterface::done1(const actionlib::SimpleClientGoalState& state, const korus_smach::MoveArmIKResultConstPtr& result)
void ArmInterface::done1()

{
//	if ( result->error.value == manipulation_comms::ManipulationErrorCodes::SUCCESS )
//	{
//		ROS_DEBUG("Korus rapi: goal returned with success");
//		sig_response.emit( 0 );
//	}
//	else
//	{
//		ROS_WARN("Korus rapi: arm goal returned with error code #");
//		sig_response.emit( result->error.value );
//	}

//	int32 SUCCESSFUL=0
//	int32 INVALID_GOAL=-1
//	int32 INVALID_JOINTS=-2
//	int32 OLD_HEADER_TIMESTAMP=-3
//	int32 PATH_TOLERANCE_VIOLATED=-4
//	int32 GOAL_TOLERANCE_VIOLATED=-5
//	int32 error_code

//	if ( result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL )
//	{
//		ROS_DEBUG("Korus rapi: goal returned with success");
//		sig_response.emit( 0 );
//	}
//	else
//	{
//		ROS_WARN("Korus rapi: arm goal returned with error code #");
//		sig_response.emit( result->error_code );
//	}
}





} // namespace qkorus
