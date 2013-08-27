/**
 * @file /qkorus/src/gripper_interface.cpp
 *
 * @brief Implementation of the Gripper (LCD monitor) position controller
 *
 * @date October 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/korus_rapi/gripper_interface.hpp"
#include <iostream>
#include <std_msgs/String.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace korus_rapi {

/*****************************************************************************
** Using
*****************************************************************************/


GripperInterface::GripperInterface() :
		joint_states_topic("/korus/joint_state/gripper"),
		action_client("/korus/gripper_controller"),
		sig_response("sig_gripper_command_interface")
{
	/*********************
	** Ros
	**********************/
	ros::NodeHandle nh("~");
	// @todo actually there are not source to read the joint state
	joint_states_subscriber = nh.subscribe(joint_states_topic,10,&GripperInterface::subscribeJointStates, this);
	enable_publisher = nh.advertise<std_msgs::String>("/korus/enable", 100);
	disable_publisher = nh.advertise<std_msgs::String>("/korus/disable", 100);


	joint_names.push_back("gripper");

	/*********************
	** Names
	**********************/
//	ros::ServiceClient mechanism_client = nh.serviceClient<device_msgs::StringJointParameters>("/korus/mechanism_model/joint_parameters");
//    device_msgs::StringJointParameters srv;
//    srv.request.name = std::string(joint_name);
//	if ( !mechanism_client.call(srv)) {
//		ROS_WARN_STREAM("Gripper Node: failed to contact the mechanism model for joint configurations.");
//	} else {
//		joint_parameters = srv.response.parameters;
//	}

	/*********************
	** Qt Model
	**********************/
}

GripperInterface::~GripperInterface() {
	joint_states_subscriber.shutdown(); // make sure we dont' do callbacks after the gui shut down.
}

/*****************************************************************************
** Ros
*****************************************************************************/

void GripperInterface::subscribeJointStates(const device_msgs::JointStateConstPtr state) {
	joint_angle = state->position;
}

/*****************************************************************************
** Power
*****************************************************************************/

void GripperInterface::enable() {
	std::cout << "Enable: " << joint_names[0] << std::endl;
	std_msgs::String name;
	name.data  = joint_names[0];
	enable_publisher.publish(name);
}

void GripperInterface::disable() {
	std::cout << "Disable: " << joint_names[0] << std::endl;
	std_msgs::String name;
	name.data  = joint_names[0];
	disable_publisher.publish(name);
}
/*****************************************************************************
** Gui Callbacks
*****************************************************************************/

/**
 * send the desired gripper angle at here
 */
void GripperInterface::send( const double & angle ) {
	if ( !action_client.waitForServer( ros::Duration(0.25)) ) {
		ROS_WARN_STREAM("Could not connect to the gripper_interface action server.");
		sig_response.emit( -1 );
		return;
	}

	enable();
	ros::Duration sleeper(0.2); sleeper.sleep();


//	manipulation_comms::JointGoal goal;
	control_msgs::FollowJointTrajectoryGoal goal;

	goal.trajectory.joint_names = joint_names;
	//goal.target.name = joint_name;

	//goal.target.position = angle;
	trajectory_msgs::JointTrajectoryPoint point; // way point
	point.positions.push_back(angle);
	point.velocities.push_back(0.5);
	point.accelerations.push_back(0);
	goal.trajectory.points.push_back(point);
	action_client.sendGoal(goal, boost::bind(&GripperInterface::done, this, _1, _2), boost::bind(&GripperInterface::active, this ), boost::bind(&GripperInterface::feedback, this ) );
}

/*****************************************************************************
** Action Callbacks
*****************************************************************************/
void GripperInterface::feedback() {
	std::cout << "Goal in progress..." << std::endl;
}

/** @brief This gets called as soon as the server moves the goal from pending to active. */
void GripperInterface::active() {
	std::cout << "Goal acknowledged." << std::endl;
}


void GripperInterface::done(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
	if ( result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL )
	{
		ROS_DEBUG("Korus rapi: gripper goal returned with success");
		sig_response.emit( 0 );
	}
	else
	{
		ROS_WARN("Korus rapi: gripper goal returned with error code #");
		sig_response.emit( result->error_code );
	}
}

} // namespace qkorus
