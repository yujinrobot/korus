/**
 * @file /qkorus/src/head_interface.cpp
 *
 * @brief Implementation of the head connections
 *
 * @date October 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/korus_rapi/head_interface.hpp"
#include <iostream>
#include <std_msgs/String.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace korus_rapi {

/*****************************************************************************
** Statics
*****************************************************************************/


/*****************************************************************************
** Implementation
*****************************************************************************/

HeadInterface::HeadInterface() :
		joint_states_topic("/korus/head_controller/state"),
		action_client("/korus/head_controller"),
		sig_response("sig_head_command_interface")
{
	/*********************
	** Ros
	**********************/
	ros::NodeHandle nh("~");
	joint_states_subscriber = nh.subscribe(joint_states_topic,10,&HeadInterface::subscribeJointStates, this);
	enable_publisher = nh.advertise<std_msgs::String>("/korus/enable", 100);
	disable_publisher = nh.advertise<std_msgs::String>("/korus/disable", 100);

	/*********************
	** Names
	**********************/
	joint_names.push_back("head_pan");
	joint_names.push_back("head_tilt");

}

HeadInterface::~HeadInterface() {
	joint_states_subscriber.shutdown(); // make sure we dont' do callbacks after the gui shut down.
}

/*****************************************************************************
** Ros
*****************************************************************************/

void HeadInterface::subscribeJointStates(const controller_msgs::JointTrajectoryControllerState& state) {

	for ( unsigned int i = 0; i < state.trajectory.actual_position.size(); ++i ) {
		joint_angles[i] = state.trajectory.actual_position[i];
	}
}
/*****************************************************************************
** Power
*****************************************************************************/

void HeadInterface::enable() {
	for ( unsigned int i = 0; i < joint_names.size(); ++i ) {
		std::cout << "Enable: " << joint_names[i] << std::endl;
		std_msgs::String name;
		name.data  = joint_names[i];
		enable_publisher.publish(name);
	}
}

void HeadInterface::disable() {
	for ( unsigned int i = 0; i < joint_names.size(); ++i ) {
		std::cout << "Disable: " << joint_names[i] << std::endl;
		std_msgs::String name;
		name.data  = joint_names[i];
		disable_publisher.publish(name);
	}
}

/*****************************************************************************
** Gui Callbacks
*****************************************************************************/

/**
 * This will be expanded more later if we need to add more than one
 * waypoint to the trajectory goal.
 */
void HeadInterface::sendTrajectory( const std::vector<double> & jointAngles )
{
	if ( !action_client.waitForServer( ros::Duration(0.25)) ) {
		ROS_WARN_STREAM("Could not connect to the head action server.");
		return;
	}

	enable();
	ros::Duration sleeper(0.2); sleeper.sleep();
//	manipulation_comms::JointTrajectoryGoal goal;
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names = joint_names;
//	goal.trajectory.header.stamp = ros::Time::now();
//	manipulation_comms::JointTrajectoryPoint point;

	trajectory_msgs::JointTrajectoryPoint point; // way point
	for ( unsigned int joint = 0; joint < joint_names.size(); ++joint ) {
		point.positions.push_back(jointAngles[joint]);
		point.velocities.push_back(0.5);
		point.accelerations.push_back(0);

	}
	goal.trajectory.points.push_back(point);
	action_client.sendGoal(goal,
			 boost::bind(&HeadInterface::done, this, _1, _2),
			boost::bind(&HeadInterface::active, this ),
			boost::bind(&HeadInterface::feedback, this ) );
}

/*****************************************************************************
** Action Callbacks
*****************************************************************************/

void HeadInterface::feedback() {
	std::cout << "Goal in progress..." << std::endl;
}

/** @brief This gets called as soon as the server moves the goal from pending to active. */
void HeadInterface::active() {
	std::cout << "Goal acknowledged." << std::endl;
}


void HeadInterface::done(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
	if ( result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL )
	{
		ROS_DEBUG("Korus rapi: head goal returned with success");
		sig_response.emit( 0 );
	}
	else
	{
		ROS_WARN("Korus rapi: head goal returned with error code #");
		sig_response.emit( result->error_code );
	}
}

} // namespace qkorus
