/**
 * @file /qkorus/include/qkorus/head_interface.hpp
 *
 * @brief Ros head connection.
 *
 * @date October
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HEAD_INTERFACE_HPP_
#define HEAD_INTERFACE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

//#include <manipulation_comms/JointTrajectoryControllerFeedback.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ecl/sigslots.hpp>
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

class HeadInterface {
public:
	/*********************
	** Typedef
	**********************/
	//typedef actionlib::SimpleActionClient<manipulation_comms::JointTrajectoryAction> ActionClient;

	typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;


	/*********************
	** C&D
	**********************/
	HeadInterface();
	~HeadInterface();

	/*********************
	** Accessors
	**********************/
	unsigned int size() { return joint_names.size(); }

	/*********************
	** Gui Callbacks
	**********************/
	void resetWaypoints();
	void sendTrajectory( const std::vector<double> & jointAngles );

	/*********************
	** Action Callbacks
	**********************/
	//Actually simple trajectory action server has not advanced version of "active", "done" and etc.
	void feedback();
	void active();
	void done(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
	void enable();
	void disable();
	double joint_angles[2];

private:

	std::vector< std::string > joint_names;
	ros::Subscriber joint_states_subscriber;
	ros::Publisher enable_publisher, disable_publisher;
	const std::string joint_states_topic;
	ActionClient action_client;
	ecl::Signal<int> sig_response;


//	void subscribeJointStates(const manipulation_comms::JointTrajectoryControllerFeedbackConstPtr state);
//	  void subscribeJointStates(const sensor_msgs::JointStateConstPtr& state);
	  void subscribeJointStates(const controller_msgs::JointTrajectoryControllerState& state);

};


}; // namespace qkorus

#endif /* HEAD_NODE_HPP_ */
