/*
 * nav_mng_node.cpp
 *
 *  Created on: Jan 30, 2012
 *      Author: jorge
 */

#include <tf/tf.h>

#include <float.h>

#include <boost/thread.hpp>

#include "nav_mng_node.h"

#define SONAR_BUFFER_SIZE  3


NavigationMng::NavigationMng() {
  bmp_id.push_back(0x0001);
  bmp_id.push_back(0x0002);
  bmp_id.push_back(0x0004);
  for (int i = 0; i < 7; i++)
    bmp_id.push_back(0x0000);

  bmp_str.push_back("Left");
  bmp_str.push_back("Center");
  bmp_str.push_back("Right");
  for (int i = 0; i < 7; i++)
    bmp_str.push_back("<Unused>");

  psd_id.push_back(0x0001);
  psd_id.push_back(0x0002);
  psd_id.push_back(0x0004);
  psd_id.push_back(0x0008);
  psd_id.push_back(0x0010);
  for (int i = 0; i < 5; i++)
    psd_id.push_back(0x0000);

  psd_str.push_back("Front right");
  psd_str.push_back("Front center");
  psd_str.push_back("Front left");
  psd_str.push_back("Back left");
  psd_str.push_back("Back right");
  for (int i = 0; i < 5; i++)
    psd_str.push_back("<Unused>");

  son_id.push_back(0x0001);
  son_id.push_back(0x0002);
  son_id.push_back(0x0004);
  son_id.push_back(0x0008);
  for (int i = 0; i < 6; i++)
    son_id.push_back(0x0000);

  son_str.push_back("Front right");
  son_str.push_back("Front left");
  son_str.push_back("Back left");
  son_str.push_back("Back right");
  for (int i = 0; i < 6; i++)
    son_str.push_back("<Unused>");

  for (int i = 0; i < 4; i++)
    sonars.push_back(INVALID_VALUE);

  fl_sonar.push_back(1);
  fr_sonar.push_back(1);
  bl_sonar.push_back(1);
  br_sonar.push_back(1);

  for (int i = 0; i < SONAR_BUFFER_SIZE; i++) {
    fl_sonar.push_back(INVALID_VALUE);
    fr_sonar.push_back(INVALID_VALUE);
    bl_sonar.push_back(INVALID_VALUE);
    br_sonar.push_back(INVALID_VALUE);
  }

  collision = 0;         // We assume we start in a safe, non-collision,
  drop_risk = 0;         // non-obstacles situation (TODO: it's really a
  obstacles = 0;         // good idea?)
  localized = 0;         // initially lost
  stop_smoothly  = false;
  lock_joystick  = false;
  joystick_input = false;
  navstack_input = false;

  // Switches initial positions:
  //  - on simulations, these are appropriated values
  //  - on real robot, will be corrected with the first sensor board message if they are wrong
  mode  = MODE_AUTO;
  speed = INVALID_VALUE; // initially speed limits are the specified on base_local_planner.yaml
  emerg = INVALID_VALUE; // the emergency button is ignored by now
};

NavigationMng::~NavigationMng() {
}

void NavigationMng::sonarsMsgCB(const sensor_board_msgs::Sonar::ConstPtr& msg)
{
  if (check_sonars == false)
    return;

  if (msg->index_fired_sonar == 1) {
    sonars[FRONT_RIGHT_SONAR] = msg->first_echo_data[FRONT_RIGHT_SONAR];
    sonars[FRONT_LEFT_SONAR]  = msg->first_echo_data[FRONT_LEFT_SONAR];

    fr_sonar[fr_sonar[0]] = msg->first_echo_data[FRONT_RIGHT_SONAR],
    fl_sonar[fl_sonar[0]] = msg->first_echo_data[FRONT_LEFT_SONAR];
    fr_sonar[0] = fr_sonar[0]%SONAR_BUFFER_SIZE + 1;
    fl_sonar[0] = fl_sonar[0]%SONAR_BUFFER_SIZE + 1;
  }
  else if (msg->index_fired_sonar == 3) {
    sonars[BACK_RIGHT_SONAR] = msg->first_echo_data[BACK_RIGHT_SONAR];
    sonars[BACK_LEFT_SONAR]  = msg->first_echo_data[BACK_LEFT_SONAR];

    br_sonar[br_sonar[0]] = msg->first_echo_data[BACK_RIGHT_SONAR],
    bl_sonar[bl_sonar[0]] = msg->first_echo_data[BACK_LEFT_SONAR];
    br_sonar[0] = br_sonar[0]%SONAR_BUFFER_SIZE + 1;
    bl_sonar[0] = bl_sonar[0]%SONAR_BUFFER_SIZE + 1;
  }
}

void NavigationMng::ooDeviceMsgCB(const sensor_board_msgs::OnOffDevice::ConstPtr& msg)
{
  if (bumpers.size() == 0) {
    // Fist on_off_dev msg; just clone its content except for the speed (see below)
    bumpers = msg->bumper;
    psd_irs = msg->bottom_psd;
    emerg   = msg->emergency_button[0];
    mode    = msg->touch[MODE_SWITCH];
  }
  else {
    // Check for bumpers status   TODO: implement escaping?
    for (unsigned int i = 0; i < msg->bumper.size() && check_bumpers == true; i++) {
      uint8_t bumper = msg->bumper[i];
      if (bumper != bumpers[i]) {
        if (bumper) {
          ROS_WARN("%s bumper hit!", bmp_str[i].c_str());

          uint8_t tmp = collision;
          collision |= bmp_id[i];

          if (collision && ! tmp) {
            ROS_WARN("Emergency stop! Control the robot manually to move to a safe pose");
            emergencyStop();
          }
        }
        else {
          ROS_INFO("%s bumper released", bmp_str[i].c_str());
          collision &= ! bmp_id[i];
        }
        bumpers[i] = bumper;
      }
    }

    // PSD sensors
    for (unsigned int i = 0; i < msg->bottom_psd.size() && check_psd_irs == true; i++) {
      uint8_t psd = msg->bottom_psd[i];
      if (psd != psd_irs[i]) {
        if (psd) {
          ROS_WARN("%s bottom PSD activated!", psd_str[i].c_str());

          uint8_t tmp = drop_risk;
          drop_risk |= psd_id[i];

          if (drop_risk && ! tmp) {
            ROS_WARN("Emergency stop! Control the robot manually to move to a safe pose");
            emergencyStop();
          }
        }
        else {
          ROS_INFO("%s bottom PSD deactivated", psd_str[i].c_str());
          drop_risk &= ! psd_id[i];
        }
        psd_irs[i] = psd;
      }
    }

    // Mode switch
    uint8_t mode_new = bool(msg->touch[MODE_SWITCH]);

    if ((mode_new != MODE_AUTO) && (mode_new != MODE_MANUAL))
      ROS_ERROR("Unknown navigation mode: %d. Ignoring", mode_new);
    else if (mode != mode_new) {
      // Slow down until we stop or user starts issuing commands;
      // must be in a thread, as it can take more than a second
      boost::thread slowDownThread(&NavigationMng::smoothStop, this);

      if (mode_new == MODE_AUTO) {
        // Nothing special to do, by now
      }
      else {
        // Cancel current goal sending an empty string
        actionlib_msgs::GoalID cmd;
        cmd.id = "";
        cancel_goal_pub.publish(cmd);

        ROS_INFO("Current goal has been cancelled");
      }

      mode = mode_new;
      ROS_INFO("Mode switch changed to %s", (mode == MODE_AUTO)?"auto":"manual");
    }
  }

  // Speed switch; must be in a thread, as the system call can take a while
  uint8_t speed_new = bool(msg->touch[SPEED_SWITCH]);

  if ((speed_new != SPEED_LOW) && (speed_new != SPEED_HIGH))
    ROS_ERROR("Unknown navigation speed: %d. Ignoring", speed_new);
  else if (speed != speed_new) {
    boost::thread maxSpeedThread(&NavigationMng::changeMaxSpeed, this,
                                 speed_new, (speed == INVALID_VALUE));
    speed = speed_new;
    ROS_INFO("Speed switch changed to %s", (speed == SPEED_LOW)?"low":"high");
  }
}

void NavigationMng::joyVelCmdCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Remote control message received
  joystick_input = true;

  if (have_nav_goal == true)
  {
    // Cancel current goal sending an empty string  TODO maybe need to wait a bit?
    actionlib_msgs::GoalID cmd;
    cmd.id = "";
    cancel_goal_pub.publish(cmd);

    ROS_INFO("Remote control message received: current goal has been cancelled");

    // TODO control if the goal has been reached, so we don't try to cancel something nonexistent
    have_nav_goal = false;
  }

  // Joystick is locked after a collision or drop risk; unlock it when
  // the risky condition disappears or the user release the joystick
  if ((joystick_input == false) || ((collision == false) && (drop_risk == false)))
    lock_joystick = false;

  // This will block till we reach the desired speed    speedCommand(*msg, JOYSTICK_FREQ);
  accelerateTo(*msg);

  // reset flag, so we can detect further remote control commands   USELESS ; a menos que los callback no tengan q terminar para llamarse de nuevo
  joystick_input = false;

  // Resend move command if...
  if ((mode == MODE_MANUAL)    &&    // we are in manual mode,
      (stop_smoothly == false) &&    // we are not stopping and
      (lock_joystick == false)) {    // we don't want to lock the joystick
;//    speedCommand(*msg, JOYSTICK_FREQ);
  }
}

void NavigationMng::navVelCmdCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  navstack_input = true;

  // Resend move command if...
  if ((mode == MODE_AUTO)      &&      // we are in automatic mode,
      (drop_risk == false)     &&      // all bottom PSD are off,
      (collision == false)     &&      // all bumpers are off,
      (localized != false)     &&      // we know where we are and
      (stop_smoothly == false)) {      // we are not stopping
;//    speedCommand(*msg, nav_ctrl_freq);
  }
  if (joystick_input == false) {
    speedCommand(*msg, nav_ctrl_freq);

    if (((msg->linear.x == 0.0) &&
         (msg->linear.y == 0.0) &&
         (msg->angular.z == 0.0)) &&
        ((last_cmd_vel.linear.x != 0.0) ||
         (last_cmd_vel.linear.y != 0.0) ||
         (last_cmd_vel.angular.z != 0.0))) {
      // This is a very special situation: nav stack sends a zero cmd_vel but due to our
      // speed smoothing we resend a non-zero cmd_vel. This is fine as long as nav stack
      // continues sending cmd_vel messages, but this is not the case when there are a
      // planning error. If we don't execute this stop, the robot will keep moving, so
      // we assume by default that that's the case; if navstack_input flag switches back
      // to true, we where wrong and we should stop stopping.
      ROS_DEBUG("<<SPECIAL CASE>> smoothStop %.2f, %.2f, %.2f  ...  %.2f, %.2f, %.2f",
              msg->linear.x, msg->linear.y, msg->angular.z,
              last_cmd_vel.linear.x, last_cmd_vel.linear.y, last_cmd_vel.angular.z);
      navstack_input = false;
      boost::thread slowDownThread(&NavigationMng::smoothStop, this);
    }
  }
}

void NavigationMng::curVelMsgCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  cur_vel = msg->twist.twist;
}

void NavigationMng::stgzPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_DEBUG("Stargazer localization received: %.2f, %.2f, %.2f",
             msg->pose.pose.position.x, msg->pose.pose.position.y,
             yaw(msg->pose.pose.orientation));
  if ((! (localized & LOCALIZED_AMCL)) ||
      ((abs((last_amcl_pose.header.stamp - msg->header.stamp).toSec())  < 0.5) &&
  //     (abs((last_amcl_init.header.stamp - msg->header.stamp).toSec())  > 5.0) &&
       ((dist(last_amcl_pose.pose.position,    msg->pose.pose.position)    > 1.5) ||
        (diff(last_amcl_pose.pose.orientation, msg->pose.pose.orientation) > 1.0)))) {
    // If amcl has not received an initial pose from the user, or it's reporting a pose
    // far away from the one reported by stargazer, initialize it with this message.
    // Note that we check timings to ensure we are not comparing with an old amcl pose

    // Check if the covariance of the pose is low enough to use it to (re)initialize amcl
    const boost::array<double, 36u>& cov = msg->pose.covariance;
    if (_max(cov[0], cov[1], cov[6], cov[7], cov[35]) > 0.1) {  // TODO: very arbitrary...
      ROS_WARN("Amcl not (re)initialized by stargazer because cov. is not low enough");
      return;
    }

    geometry_msgs::PoseWithCovarianceStamped pose = *msg;
    pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
    init_pose_pub.publish(pose);
    last_amcl_init.header = msg->header;
    last_amcl_init.pose = msg->pose.pose;

    localized |= LOCALIZED_AMCL;

    ROS_WARN("Amcl (re)initialized by stargazer with pose: %.2f, %.2f, %.2f",
              msg->pose.pose.position.x, msg->pose.pose.position.y, yaw(msg->pose.pose.orientation));
    ROS_DEBUG("DIST = %.2f, DIFF = %.2f, TIME = %f",    // explain the reason
               dist(last_amcl_pose.pose.position,     msg->pose.pose.position),
               diff(last_amcl_pose.pose.orientation,  msg->pose.pose.orientation),
                   (last_amcl_pose.header.stamp     - msg->header.stamp).toSec());
  }

  localized |= LOCALIZED_STGZ;
}

void NavigationMng::amclPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  last_amcl_pose.header = msg->header;
  last_amcl_pose.pose = msg->pose.pose;

  // Amcl localization received; check how good it is
  const boost::array<double, 36u>& cov = msg->pose.covariance;
  double max = _max(cov[0], cov[1], cov[6], cov[7], cov[35]);

  if (max > 2.0) {  // this is really a lot, so this case will only arise in extreme circumstances
    if (localized & LOCALIZED_AMCL) {
      ROS_WARN("Amcl is providing very inaccurate localization (max. cov. = %f)", max);
      localized &= ~LOCALIZED_AMCL;

      // Forbid navigation if amcl is really, really lost, and stargazer never saw a landmark
      if ((localized == false) && (mode == MODE_AUTO)) {
        // If we are in auto mode, we'll stop resending nav stack commands,
        // so the robot can keep moving if it was doing so; just stop it!
        boost::thread slowDownThread(&NavigationMng::smoothStop, this);
      }
    }
  }
}

void NavigationMng::newGoalMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // New goal designated; we just capture these events to provide log information

  if (mode != MODE_AUTO) {
    ROS_WARN("New goal designated while not in automatic navigation mode");
  }
  else if (drop_risk == true) {
    ROS_WARN("Automatic navigation not allowed while the robot is on dropping risk");
  }
  else if (collision == true) {
    ROS_WARN("Automatic navigation not allowed while the robot is on collision");
  }
  else if (localized == false) {
    ROS_WARN("Automatic navigation not allowed while the robot is not localized");
  }
  else {
    ROS_INFO("New goal designated: %.2f, %.2f, %.2f",
              msg->pose.position.x, msg->pose.position.y, yaw(msg->pose.orientation));
  }

  have_nav_goal = true;
}

void NavigationMng::initPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Amcl (re)initialized by someone (cannot know who); consider it as localized
  localized |= LOCALIZED_AMCL;
}

//void NavigationMng::feedbackMsgCB(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
//{
//
//}
//
//void NavigationMng::glResultMsgCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
//{
//
//}

void NavigationMng::glStatusMsgCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{

}

bool NavigationMng::speedCommand(geometry_msgs::Twist cmd_vel, double cmd_freq)
{
  // Check for obstacles ahead; only bound speed if the way is clear
  if (checkSonars(cmd_vel) == false)
    boundSpeed(cmd_vel, cmd_freq);

  cmd_vel_pub.publish(cmd_vel);

  last_cmd_vel = cmd_vel;

  return true;
}

void NavigationMng::emergencyStop()
{
  // Sharp stop; to use only in extreme cases, as a bumper hit or drop risk
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub.publish(cmd_vel);

  last_cmd_vel = cmd_vel;

  // On manual mode, forbid user command until he release the joystick
  // or the risky condition disappears
  lock_joystick = true;
}

bool NavigationMng::smoothStop()
{
  // Slow down with the maximum possible acceleration; can be used also
  // to accelerate, but I cannot see in which case this can be useful
  double frequency = 30.0; // 30 Hz, same as joystick commands

  //geometry_msgs::Twist::Ptr cmd_vel(new geometry_msgs::Twist);
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x  = 0.0;
  cmd_vel.linear.y  = 0.0;
  cmd_vel.angular.z = 0.0;

  // Try to estimate how many steps we will need fot the acceleration, so we
  // exit even if something goes grong
  int estim_steps =
      ceil(_max(abs(last_cmd_vel.linear.x  / decel_lim_x) * frequency,
                abs(last_cmd_vel.linear.y  / decel_lim_y) * frequency,
                abs(last_cmd_vel.angular.z / decel_lim_t) * frequency));

  if (estim_steps == 0)
    // Don't need to accelerate? could be a warning...
    return true;

  ROS_DEBUG("Stopping from %.2f, %.2f, %.2f m/s in %d steps at %.2f, %.2f, %.2f m/s^2",
             last_cmd_vel.linear.x, last_cmd_vel.linear.y, last_cmd_vel.angular.z,
             estim_steps, decel_lim_x, decel_lim_y, decel_lim_t);

  stop_smoothly = true;

  for (int i = 0; i < (int)estim_steps*1.5; i++) {
    // Verify that the user is not moving the joystick while in manual
    // mode. This can provoke a sharp motion, but... user rules
    if ((joystick_input == true) && (mode == MODE_MANUAL)) {
      ROS_INFO("Acceleration aborted, as user is issuing joystick commands");
      stop_smoothly = false;
      return true;
    }

    // Verify that nav stack is not issuing commands while we are in automatic
    // mode and reasonably well localized; otherwise, abort the acceleration
    if ((navstack_input == true) && (mode == MODE_AUTO) && (localized == true)) {
      ROS_INFO("Acceleration aborted, as nav stack issuing speed commands");
      stop_smoothly = false;
      return true;
    }

    speedCommand(cmd_vel, frequency);

    ROS_DEBUG("SMOOTH STOP (%d)  %.2f, %.2f, %.2f m/s", i,
        last_cmd_vel.linear.x, last_cmd_vel.linear.y, last_cmd_vel.angular.z);

    if ((last_cmd_vel.linear.x == 0.0) &&
        (last_cmd_vel.linear.y == 0.0) &&
        (last_cmd_vel.angular.z == 0.0)) {
      stop_smoothly = false;
      return true;
    }

    // WARN: this sleep implies that this method should be always called in its own thread
    ros::Duration(1.0/frequency).sleep();
  }

  // Something went wrong; just stop in any way!
  ROS_WARN("Smooth stop failed after %d steps; EMERGENCY STOP!", int(estim_steps*1.5));
  stop_smoothly = false;
  emergencyStop();

  return false;
}

bool NavigationMng::boundSpeed(geometry_msgs::Twist& cmd_vel, double cmd_freq)
{
  if (bound_speed == false)
    return false;

  // Ensure we don't exceed the acceleration limits; for each dof, we calculate the
  // commanded velocity increment and the maximum allowed increment (i.e. acceleration)
  double cmd_vel_inc, max_vel_inc;
  double period = 1.0/cmd_freq; // 30 Hz for joystick, around 5 Hz for ros navigation stack

  bool bounded = false;

  cmd_vel_inc = cmd_vel.linear.x - last_cmd_vel.linear.x;
  if (cur_vel.linear.x*cmd_vel.linear.x < 0.0)
    max_vel_inc = decel_lim_x*period;   // countermarch
  else
    max_vel_inc = ((cmd_vel_inc*cmd_vel.linear.x > 0.0)?accel_lim_x:decel_lim_x)*period;
  if (abs(cmd_vel_inc) > max_vel_inc) {
    cmd_vel.linear.x = last_cmd_vel.linear.x + sign(cmd_vel_inc)*max_vel_inc;
    bounded = true;
  }

  cmd_vel_inc = cmd_vel.linear.y - last_cmd_vel.linear.y;
  if (cur_vel.linear.y*cmd_vel.linear.y < 0.0)
    max_vel_inc = decel_lim_y*period;   // countermarch
  else
    max_vel_inc = ((cmd_vel_inc*cmd_vel.linear.y > 0.0)?accel_lim_y:decel_lim_y)*period;
  if (abs(cmd_vel_inc) > max_vel_inc) {
    cmd_vel.linear.y = last_cmd_vel.linear.y + sign(cmd_vel_inc)*max_vel_inc;
    bounded = true;
  }

  cmd_vel_inc = cmd_vel.angular.z - last_cmd_vel.angular.z;
  if (cur_vel.angular.z*cmd_vel.angular.z < 0.0)
    max_vel_inc = decel_lim_t*period;   // countermarch
  else
    max_vel_inc = ((cmd_vel_inc*cmd_vel.angular.z > 0.0)?accel_lim_t:decel_lim_t)*period;
  if (abs(cmd_vel_inc) > max_vel_inc) {
    cmd_vel.angular.z = last_cmd_vel.angular.z + sign(cmd_vel_inc)*max_vel_inc;
    bounded = true;
  }

  return bounded;
}

bool NavigationMng::checkSonars(geometry_msgs::Twist& cmd_vel)
{
  if (check_sonars == false)
    return false;

  // Slow down (and stop if needed) when the sonars report we approach an obstacle
  if (cmd_vel.linear.x > 0.0)
  {
    double dist = min(median(fr_sonar), median(fl_sonar))/100.0;
    if (dist <= OBSTACLE_MAX_DIST)
    {
      double speed_modif = sqrt(max(dist - OBSTACLE_MIN_DIST, 0.0)/OBSTACLE_MAX_DIST);
      if (fr_sonar[0]%SONAR_BUFFER_SIZE == 1) // Little trick to make this warn less verbose
        ROS_WARN("Front sonars detect an obstacle at %.2f m. %f", dist, speed_modif);
      cmd_vel.linear.x *= speed_modif;
      return true;
    }
  }
  else if (cmd_vel.linear.x < 0.0)
  {
    double dist = min(median(br_sonar), median(bl_sonar))/100.0;
    if (dist <= OBSTACLE_MAX_DIST)
    {
      double speed_modif = sqrt(max(dist - OBSTACLE_MIN_DIST, 0.0)/OBSTACLE_MAX_DIST);
      if (br_sonar[0]%SONAR_BUFFER_SIZE == 1) // Little trick to make this warn less verbose
        ROS_WARN("Back sonars detect an obstacle at %.2f m. %f", dist, speed_modif);
      cmd_vel.linear.x *= speed_modif;
      return true;
    }
  }

  return false;
}

int NavigationMng::changeMaxSpeed(uint8_t speed_new, bool first_call)
{
  int status = 0;
  char system_cmd[256];

  // Call dynamic_reconfigure to change max_vel_x and max_vel_theta parameters values
  if (speed_new == SPEED_HIGH)
  {
    snprintf(system_cmd, 256,
             "rosrun dynamic_reconfigure dynparam set %s \"{ %s: %f, %s: %f }\"",
             move_base_node_name.c_str(),
             max_linear_speed_param.c_str(), high_max_linear_speed,
             max_angular_speed_param.c_str(), high_max_angular_speed);

    status = system(system_cmd);
  }
  else
  {
    snprintf(system_cmd, 256,
             "rosrun dynamic_reconfigure dynparam set %s \"{ %s: %f, %s: %f }\"",
             move_base_node_name.c_str(),
             max_linear_speed_param.c_str(), low_max_linear_speed,
             max_angular_speed_param.c_str(), low_max_angular_speed);

    status = system(system_cmd);
  }

  if (first_call == false)
  {
    // Not the first on_off_dev message
    if (status == 0)
      ROS_INFO("Speed switch changed to %s", (speed_new == SPEED_LOW)?"low":"high");
    else
      ROS_ERROR("Change speed failed due to an error on system call (%d/%d)",
                status, WEXITSTATUS(status));
  }

  return status;
}

int NavigationMng::init(ros::NodeHandle& nh)
{
  // Parameters
  nh.getParam("navigation_mng/bound_speed", bound_speed);
  nh.getParam("navigation_mng/check_sonars", check_sonars);
  nh.getParam("navigation_mng/check_bumpers", check_bumpers);
  nh.getParam("navigation_mng/check_psd_irs", check_psd_irs);
  nh.getParam("navigation_mng/check_localized", check_localized);

  nh.getParam("move_base_node/TrajectoryPlannerROS/acc_lim_x", accel_lim_x);
  nh.getParam("move_base_node/TrajectoryPlannerROS/acc_lim_y", accel_lim_y);
  nh.getParam("move_base_node/TrajectoryPlannerROS/acc_lim_theta", accel_lim_t);
  nh.getParam("move_base_node/controller_frequency", nav_ctrl_freq);

  if (check_localized == false)
  {
    // Ignore whether we are localized; useful in local navigation or mapping
    localized |= LOCALIZED_FAKE;
  }

  // Deceleration must be more aggressive to compensate inertia    NOT IN KORUS
//  decel_lim_x = 2.0*accel_lim_x;
//  decel_lim_y = 2.0*accel_lim_y;
//  decel_lim_t = 4.5*accel_lim_t;
  decel_lim_x = accel_lim_x;
  decel_lim_y = accel_lim_y;
  decel_lim_t = accel_lim_t;

  // Publishers and subscriptors
//  sonars_sub  = nh.subscribe("sonars",      1, &NavigationMng::sonarsMsgCB,   this);
  //device_sub  = nh.subscribe("on_off_dev",  1, &NavigationMng::ooDeviceMsgCB, this);
  stargz_sub  = nh.subscribe("stargazer",   1, &NavigationMng::stgzPoseMsgCB, this);
  amcl_p_sub  = nh.subscribe("amcl_pose",   1, &NavigationMng::amclPoseMsgCB, this);
  init_p_sub  = nh.subscribe("amcl_init",   1, &NavigationMng::initPoseMsgCB, this);
  cur_vel_sub = nh.subscribe("odometry",    1, &NavigationMng::curVelMsgCB,   this);
  joy_vel_sub = nh.subscribe("joy_cmd_vel", 1, &NavigationMng::joyVelCmdCB,   this);
  nav_vel_sub = nh.subscribe("nav_cmd_vel", 1, &NavigationMng::navVelCmdCB,   this);
  n_goal_sub  = nh.subscribe("new_goal",    1, &NavigationMng::newGoalMsgCB,  this);

  cmd_vel_pub     = nh.advertise <geometry_msgs::Twist>                     ("mng_cmd_vel",  1);
  init_pose_pub   = nh.advertise <geometry_msgs::PoseWithCovarianceStamped> ("initial_pose", 1);
  cancel_goal_pub = nh.advertise <actionlib_msgs::GoalID>                   ("cancel_goal",  1);

  ROS_INFO("Navigation manager node successfully initialized");

  return 0;
}

int NavigationMng::spin() {

  ros::spin();

  return 0;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_mng");

  ros::NodeHandle nh;

  NavigationMng node;
  node.init(nh);
  node.spin();

  return 0;
}



// TODO old version using odometry;  not used, and probably will be removed
bool NavigationMng::accelerateTo_2(double target_vel_x,
                                 double target_vel_y,
                                 double target_vel_t)
{
  // Slow down with the maximum possible acceleration; can be used also
  // to decelerate, but I cannot see in which case this can be useful
  double period = 0.1; // 10 Hz, same as stage odometry rate

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  // Try to estimate how many steps we will need fot the acceleration, so we
  // exit even if something goes grong
  int estim_steps =
      ceil(_max(abs(target_vel_x - cur_vel.linear.x)/(accel_lim_x*period),
                abs(target_vel_y - cur_vel.linear.y)/(accel_lim_y*period),
                abs(target_vel_t - cur_vel.angular.z)/(accel_lim_t*period)));

  if (estim_steps == 0)
    // Don't need to accelerate? could be a warning...
    return true;

  ROS_ERROR("Accelerating from %.2f, %.2f, %.2f to %.2f, %.2f, %.2f m/s in %d steps at %.2f, %.2f, %.2f m/s^2",
            cur_vel.linear.x, cur_vel.linear.y, cur_vel.angular.z,
            target_vel_x, target_vel_y, target_vel_t, estim_steps,
            accel_lim_x, accel_lim_y, accel_lim_t);

  for (int i = 0; i < (int)estim_steps*1.5; i++) {
    // Verify that the user is not moving the joystick while in manual mode
    // This can provoke a sharp motion, but... user rules
    if ((joystick_input == true) && (mode == MODE_MANUAL)) {
      ROS_INFO("Acceleration aborted, as user is issuing joystick commands");
      return true;
    }

    // Verify that nav stack is not issuing commands while in automatic mode
    if ((navstack_input == true) && (mode == MODE_AUTO)) {
      ROS_INFO("Acceleration aborted, as nav stack issuing speed commands");
      return true;
    }

    if (abs(target_vel_x - cur_vel.linear.x) <= accel_lim_x*period)
      cmd_vel.linear.x = target_vel_x;
    else
      cmd_vel.linear.x = cur_vel.linear.x
                       + sign(target_vel_x - cur_vel.linear.x)*accel_lim_x*period;

    if (abs(target_vel_y - cur_vel.linear.y) <= accel_lim_y*period)
      cmd_vel.linear.y = target_vel_y;
    else
      cmd_vel.linear.y = cur_vel.linear.y
                       + sign(target_vel_y - cur_vel.linear.y)*accel_lim_y*period;

    if (abs(target_vel_t - cur_vel.angular.z) <= accel_lim_t*period)
      cmd_vel.angular.z = target_vel_t;
    else
      cmd_vel.angular.z = cur_vel.angular.z
                        + sign(target_vel_t - cur_vel.angular.z)*accel_lim_t*period;

    cmd_vel_pub.publish(cmd_vel);

    last_cmd_vel = cmd_vel;

    ROS_ERROR(" %d >>> SET SPEED TO %.2f, %.2f, %.2f m/s", i,
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    if ((cmd_vel.linear.x == target_vel_x) &&
        (cmd_vel.linear.y == target_vel_y) &&
        (cmd_vel.angular.z == target_vel_t))
      return true;

    ros::Duration(period).sleep();
  }

  // Something went wrong; just send the desired speed
  ROS_ERROR("Accelerate to %.2f, %.2f, %.2f failed after %d steps",
            target_vel_x, target_vel_y, target_vel_t, estim_steps*2);

  cmd_vel.linear.x  = target_vel_x;
  cmd_vel.linear.y  = target_vel_y;
  cmd_vel.angular.z = target_vel_t;
  cmd_vel_pub.publish(cmd_vel);

  last_cmd_vel = cmd_vel;

  return false;
}

// old version replaced by smoothStop, which reuse boundSpeed; probably will be discarded
bool NavigationMng::accelerateTo(const geometry_msgs::Twist& target_vel)
{
  // Accelerate to target velocity without violating the same limits that local planner uses
  double period = 0.1; // 10 Hz, same as stage odometry rate

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x  = 0.0;
  cmd_vel.linear.y  = 0.0;
  cmd_vel.angular.z = 0.0;

  // Estimate how many steps we will need to accelerate, so we exit even if something goes wrong
  int estim_steps =
      ceil(_max(abs(target_vel.linear.x  - last_cmd_vel.linear.x)  / (accel_lim_x*period),
                abs(target_vel.linear.y  - last_cmd_vel.linear.y)  / (accel_lim_y*period),
                abs(target_vel.angular.z - last_cmd_vel.angular.z) / (accel_lim_t*period)));

  if (estim_steps == 0)
    // Don't need to accelerate? could be a warning...
    return true;

  ROS_INFO("Accelerating from %.2f, %.2f, %.2f to %.2f, %.2f, %.2f m/s in %d steps at %.2f, %.2f, %.2f m/s^2",
            last_cmd_vel.linear.x, last_cmd_vel.linear.y, last_cmd_vel.angular.z,
            target_vel.linear.x, target_vel.linear.y, target_vel.angular.z, estim_steps,
            accel_lim_x, accel_lim_y, accel_lim_t);

  // Ensure we don't exceed the acceleration limits; for each dof, we calculate the
  // commanded velocity increment and the maximum allowed increment (i.e. acceleration)
  stop_smoothly = true;

  for (int i = 0; i < (int)estim_steps; i++) {
/*
    // Verify that the user is not moving the joystick while in manual mode
    // This can provoke a sharp motion, but... user rules
    if ((joystick_input == true) ) {//&& (mode == MODE_MANUAL)) {
      ROS_INFO("Acceleration aborted, as user is issuing joystick commands");
      stop_smoothly = false;
      is not concurrent, so this don't make sense; rethink... break loop   return true;
    }

    // Verify that nav stack is not issuing commands while in automatic mode
    if ((navstack_input == true) && (mode == MODE_AUTO)) {
      ROS_INFO("Acceleration aborted, as nav stack issuing speed commands");
      stop_smoothly = false;
      return true;
    }*/



    if (abs(target_vel.linear.x - last_cmd_vel.linear.x) <= accel_lim_x*period)
      cmd_vel.linear.x = target_vel.linear.x;
    else
      cmd_vel.linear.x = last_cmd_vel.linear.x
                       + sign(target_vel.linear.x - last_cmd_vel.linear.x)*accel_lim_x*period;

    if (abs(target_vel.linear.y - last_cmd_vel.linear.y) <= accel_lim_y*period)
      cmd_vel.linear.y = target_vel.linear.y;
    else
      cmd_vel.linear.y = last_cmd_vel.linear.y
                       + sign(target_vel.linear.y - last_cmd_vel.linear.y)*accel_lim_y*period;

    if (abs(target_vel.angular.z - last_cmd_vel.angular.z) <= accel_lim_t*period)
      cmd_vel.angular.z = target_vel.angular.z;
    else
      cmd_vel.angular.z = last_cmd_vel.angular.z
                        + sign(target_vel.angular.z - last_cmd_vel.angular.z)*accel_lim_t*period;

    cmd_vel_pub.publish(cmd_vel);

    last_cmd_vel = cmd_vel;

    ROS_DEBUG("Step %d >>> SET SPEED TO %.2f, %.2f, %.2f m/s", i, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    if ((cmd_vel.linear.x  == target_vel.linear.x) &&
        (cmd_vel.linear.y  == target_vel.linear.y) &&
        (cmd_vel.angular.z == target_vel.angular.z)) {
      stop_smoothly = false;
      return true;
    }

    usleep(period*1000000);
  }

  // Something went wrong; just send the desired speed, even with the 50% extra steps
  ROS_WARN("Accelerate to %.2f, %.2f, %.2f failed after %d steps",
           target_vel.linear.x, target_vel.linear.y, target_vel.angular.z, estim_steps);

  cmd_vel.linear.x  = target_vel.linear.x;
  cmd_vel.linear.y  = target_vel.linear.y;
  cmd_vel.angular.z = target_vel.angular.z;
  cmd_vel_pub.publish(cmd_vel);

  last_cmd_vel = cmd_vel;

  stop_smoothly = false;
  return false;
}
