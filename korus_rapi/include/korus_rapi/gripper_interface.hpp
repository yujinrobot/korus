/**
 * @file /qkorus/include/qkorus/gripper_interface.hpp
 *
 * @brief Ros head connection.
 *
 * @date October
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef GRIPPER_INTERFACE_HPP_
#define GRIPPER_INTERFACE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <vector>
#include <device_msgs/JointParameters.h>
#include <device_msgs/JointState.h>
//#include <control_msgs/JointAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ecl/sigslots.hpp>



//#include <manipulation_comms/JointTrajectoryControllerFeedback.h>
#include <control_msgs/JointTrajectoryAction.h>

#include <ecl/threads/thread.hpp>


//#include <QObject>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/JointState.h>
#include <device_msgs/EmptyJointConfiguration.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_msgs/JointTrajectoryControllerState.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace korus_rapi {

/*****************************************************************************
** Using
*****************************************************************************/

class GripperInterface {
public:
	/*********************
	** Typedef
	**********************/
	//typedef actionlib::SimpleActionClient<manipulation_comms::JointAction> ActionClient;
	typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
	/*********************
	** C&D
	**********************/
	GripperInterface();
	~GripperInterface();

	/*********************
	** Ranges
	**********************/
	double minRange() { return joint_parameters.limit_min; }
	double maxRange() { return joint_parameters.limit_max; }

	/*********************
	** Models
	**********************/

	/*********************
	** Gui Callbacks
	**********************/
	void send( const double & angle );	// pan and tilt

	/*********************
	** Action Callbacks
	**********************/
	//Actually simple trajectory action server has not advanced version of "active", "done" and etc.
	void feedback();
	void active();
	void done(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
	void enable();
	void disable();

	double joint_angle;

private:
	std::vector< std::string > joint_names;
	ros::Subscriber joint_states_subscriber;
	ros::Publisher enable_publisher, disable_publisher;
	device_msgs::JointParameters joint_parameters;
	const std::string joint_states_topic;
	ActionClient action_client;
	void subscribeJointStates(const device_msgs::JointStateConstPtr state);
    //void subscribeJointStates(const manipulation_comms::JointTrajectoryControllerState& state);

	ecl::Signal< int > sig_response;
};


} // namespace qkorus


#endif /* GRIPPER_INTERFACE_HPP_ */
