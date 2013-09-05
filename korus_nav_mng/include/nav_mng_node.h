/*
 * nav_mng_node.h
 *
 *  Created on: Jan 30, 2012
 *      Author: jorge
 */

#ifndef NAV_MNG_NODE_H_
#define NAV_MNG_NODE_H_


#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sensor_board_msgs/Sonar.h>
#include <sensor_board_msgs/OnOffDevice.h>

using namespace std;

// Buttons and sensors dictionaries
#define LEFT_BUMPER           0
#define CENTER_BUMPER         1
#define RIGHT_BUMPER          2

#define FRONT_RIGHT_PSD       0
#define FRONT_CENTER_PSD      1
#define FRONT_LEFT_PSD        2
#define BACK_LEFT_PSD         3
#define BACK_RIGHT_PSD        4

#define FRONT_RIGHT_SONAR     0
#define FRONT_LEFT_SONAR      1
#define BACK_LEFT_SONAR       2
#define BACK_RIGHT_SONAR      3

#define MODE_SWITCH           3
#define SPEED_SWITCH          4

#define MODE_MANUAL           0
#define MODE_AUTO             1

#define SPEED_LOW             0
#define SPEED_HIGH            1

#define LOCALIZED_STGZ   0x0001
#define LOCALIZED_AMCL   0x0002
#define LOCALIZED_FAKE   0x0004   // Ignore whether we are localized (local nav. or mapping

#define INVALID_VALUE       255

#define JOYSTICK_FREQ        30

#define OBSTACLE_MIN_DIST   0.4   // Minimun distance allowed to an obstacle, in m
#define OBSTACLE_MAX_DIST   0.8   // Limit of the influence of an obstacle, in m


class NavigationMng {
public:

  /*********************
  ** Initialization
  **********************/
  NavigationMng();
  ~NavigationMng();
  int init(ros::NodeHandle& nh);

  /*********************
  ** Runtime
  **********************/
  int spin();

private:
  double accel_lim_x, decel_lim_x;
  double accel_lim_y, decel_lim_y;
  double accel_lim_t, decel_lim_t;

  double nav_ctrl_freq;

  double low_max_linear_speed;
  double high_max_linear_speed;
  double low_max_angular_speed;
  double high_max_angular_speed;
  string max_linear_speed_param;
  string max_angular_speed_param;
  string move_base_node_name;

  vector<uint8_t> bmp_id;
  vector<uint8_t> psd_id;
  vector<uint8_t> son_id;
  vector<string>  bmp_str;
  vector<string>  psd_str;
  vector<string>  son_str;

  vector<uint8_t> bumpers;
  vector<uint8_t> psd_irs;
  vector<uint8_t> sonars;
  vector<uint8_t> fl_sonar, fr_sonar;
  vector<uint8_t> bl_sonar, br_sonar;

  bool bound_speed;
  bool check_sonars;
  bool check_bumpers;
  bool check_psd_irs;
  bool check_localized;

  bool stop_smoothly;
  bool lock_joystick;
  bool joystick_input;
  bool navstack_input;
  bool motors_enabled;
  bool have_nav_goal;
  uint16_t collision;
  uint16_t drop_risk;
  uint16_t obstacles;
  uint16_t localized;

  uint8_t mode;
  uint8_t speed;
  uint8_t emerg;

  geometry_msgs::Twist            cur_vel;
  geometry_msgs::Twist       last_cmd_vel;
  geometry_msgs::PoseStamped last_amcl_pose;
  geometry_msgs::PoseStamped last_amcl_init;

  ros::Subscriber sonars_sub;
  ros::Subscriber device_sub;
  ros::Subscriber stargz_sub;  /**< Robot's world pose estimated by stargazer, with cov.  */
  ros::Subscriber amcl_p_sub;  /**< Robot's world pose estimated by amcl, with covariance */
  ros::Subscriber init_p_sub;  /**< Mean and cov. to (re)initialize amcl particle filter  */
  ros::Subscriber feedbk_sub;  /**< Feedback on current position of the base in the world */
  ros::Subscriber status_sub;  /**< Status information on the goals sent to the move_base */
  ros::Subscriber result_sub;  /**< Result is empty for the move_base action */
  ros::Subscriber n_goal_sub;  /**< A goal for move_base to pursue in the world */
  ros::Subscriber joy_vel_sub;
  ros::Subscriber nav_vel_sub;
  ros::Subscriber cur_vel_sub;
  ros::Subscriber act_goal_sub;

  ros::Publisher cmd_vel_pub;
  ros::Publisher init_pose_pub;
  ros::Publisher cancel_goal_pub;
  ros::Publisher motor_enable_pub;
  ros::Publisher motor_disable_pub;

  void sonarsMsgCB(const sensor_board_msgs::Sonar::ConstPtr& msg);
  void newGoalMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void ooDeviceMsgCB(const sensor_board_msgs::OnOffDevice::ConstPtr& msg);
  void amclPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void stgzPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void initPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
//  void feedbackMsgCB(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
//  void glResultMsgCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void glStatusMsgCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
  void newGoalActionCB(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

  void curVelMsgCB(const nav_msgs::Odometry::ConstPtr& msg);
  void joyVelCmdCB(const geometry_msgs::Twist::ConstPtr& msg);
  void navVelCmdCB(const geometry_msgs::Twist::ConstPtr& msg);

  bool speedCommand(geometry_msgs::Twist cmd_vel, double cmd_freq);
  bool checkSonars(geometry_msgs::Twist& cmd_vel);
  bool boundSpeed(geometry_msgs::Twist& cmd_vel, double cmd_freq);

  bool smoothStop();
  void emergencyStop();
  int  changeMaxSpeed(uint8_t speed_new, bool first_call);

  bool accelerateTo(const geometry_msgs::Twist& target_vel);
  bool accelerateTo_2(double target_vel_x, double target_vel_y, double target_vel_t);

  double sign(double x)                                          { return x < 0.0 ? -1.0 : +1.0; }

  double _max(double a, double b, double c)                      { return max( max(a, b), c); }
  double _max(double a, double b, double c, double d)            { return max(_max(a, b, c), d); }
  double _max(double a, double b, double c, double d, double e)  { return max(_max(a, b, c, d), e); }

  // maximum ignoring sign
  double maxis(double a, double b)                               { return max(abs(a), abs(b)); }
  double maxis(double a, double b, double c)                     { return maxis(maxis(a, b), c); }
  double maxis(double a, double b, double c, double d)           { return maxis(maxis(a, b, c), d); }
  double maxis(double a, double b, double c, double d, double e) { return maxis(maxis(a, b, c, d), e); }

  double  yaw(geometry_msgs::Quaternion q) { return mod((q.z > 0)?2*acos(q.w):2*(M_PI - acos(q.w))); }

  double diff(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b) { return abs(yaw(a) - yaw(b)); }

  double dist(geometry_msgs::Point a, geometry_msgs::Point b) {
    return sqrt(pow((b.x - a.x), 2) + pow((b.y - a.y), 2));
  }

  double mod(double a) {
    while (a > +M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  uint8_t median(vector<uint8_t> values) {
    // Return the median element of an integers vector ignoring the first element
    nth_element(values.begin() + 1, values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  }

};

#endif /* NAV_MNG_NODE_H_ */
