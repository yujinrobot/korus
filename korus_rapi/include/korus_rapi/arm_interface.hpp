/**
 * @file /qkorus/include/qkorus/arm_interface.hpp
 *
 * @brief Ros arm connection.
 *
 * @date October
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ARM_INTERFACE_HPP_
#define ARM_INTERFACE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <vector>
#include <device_msgs/JointParameters.h>
//#include <manipulation_comms/JointTrajectoryControllerFeedback.h>
//#include <manipulation_comms/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <ecl/sigslots.hpp>


#include <sensor_msgs/JointState.h>

//#include <manipulation_comms/JointTrajectoryControllerFeedback.h>

#include <ecl/threads/thread.hpp>


//#include <QObject>
#include <boost/shared_ptr.hpp>


#include <device_msgs/EmptyJointConfiguration.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_msgs/JointTrajectoryControllerState.h>

//#include <korus_smach/MoveArmIKAction.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace korus_rapi {

/*****************************************************************************
** Using
*****************************************************************************/

class ArmInterface {
public:
	/*********************
	** Typedef
	**********************/
	//typedef actionlib::SimpleActionClient<manipulation_comms::MoveArmAction> ActionClient;
	typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
//	typedef actionlib::SimpleActionClient<korus_smach::MoveArmIKAction> ActionClient1;

	/*********************
	** C&D
	**********************/
	ArmInterface();
	~ArmInterface();

	/*********************
	** Accessors
	**********************/
	unsigned int size() { return joint_names.size(); }

	/*********************
	** Gui Callbacks
	**********************/
	void enable();
	void disable();
	void solveTargetPose( std::vector<double> & targetPose );
	void sendTrajectory(std::vector<double> & jointAngles, unsigned int & mode);

	/*********************
	** Action Callbacks
	**********************/
	void feedback();
	void active();
	void done(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);


	void feedback1();
	void active1();

//	void done1(const actionlib::SimpleClientGoalState& state, const korus_smach::MoveArmIKResultConstPtr& result);
	void done1();
	// for rapi
	double joint_angles[7];
	double end_effector_pose[7];

private:
	std::vector<std::string>  joint_names;
	std::vector< device_msgs::JointParameters > joint_configuration;
	ros::Subscriber arm_controller_state_subscriber;

	const std::string arm_controller_state_topic;
	const std::string enable_topic, disable_topic;
	const std::string get_fk_topic, get_ik_topic, get_fk_solver_info_topic, get_ik_solver_info_topic;

	ros::Publisher enable_publisher, disable_publisher;
	ActionClient action_client;
//	ActionClient1 action_client1;


//	void subscribeArmControllerState(const manipulation_comms::JointTrajectoryControllerFeedbackConstPtr state);
	void subscribeArmControllerState(const controller_msgs::JointTrajectoryControllerState& state);

	// why should i set the joint names for ik solver; does not make sense
	// response woulbe be member in order to avoid the re-assignment of joint and link names.
	// this will work even if joint names are changed
	// however you should call the ik_solver_info.!!
//	kinematics_msgs::GetKinematicSolverInfo::Response response;
	std::vector<double> ik_result;
	bool move_arm_connected;
	bool inverse_kinematics_connected;
	ecl::Signal<int> sig_response;


};

} // namespace qkorus

#endif /* ARM_interface_HPP_ */
