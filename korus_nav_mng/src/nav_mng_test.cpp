/*
 * nav_mng_test.cpp
 *
 *  Created on: Jan 30, 2012
 *      Author: jorge
 */

#include <termios.h> // for keyboard input

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stargazer_msgs/LocationData.h>
#include <sensor_board_msgs/OnOffDevice.h>

void dumpToScreen();
void kbInputLoop();

ros::Publisher device_pub;
ros::Publisher sgdata_pub;
ros::Publisher sgpose_pub;

long int sg_pose_sec = 0;

sensor_board_msgs::OnOffDevice msg;

/**
 * This program emulates the korus's sensor integration board for debug pourposes
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "board_emulator");

  ros::NodeHandle n;

  device_pub =
      n.advertise<sensor_board_msgs::OnOffDevice>("OnOffDevice", 1000);

  sgdata_pub =
      n.advertise <stargazer_msgs::LocationData>("/korus/stargazer/LocationData",  1);

  sgpose_pub =
      n.advertise <geometry_msgs::PoseWithCovarianceStamped>("/korus/stargazer/pose",  1);

  ros::Rate loop_rate(10);

  int count = 0;

  msg.bottom_psd.assign(6, 0);
  msg.bumper.assign(4, 0);
  msg.emergency_button.assign(1, 0);
  msg.touch.assign(5, 0);

  boost::thread kbInThread(kbInputLoop);

  while (ros::ok())
  {
    device_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


/**
 * @brief The worker thread function that accepts input keyboard commands.
 */
void kbInputLoop() {
  int kfd = 0;
  struct termios cooked, raw;
  tcgetattr(kfd,&cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));

  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  bool quit_requested = false;

  char c;
  while ( !quit_requested ) {
    dumpToScreen();

    if (read(kfd, &c, 1) < 0) {
      perror("read char failed():");
      exit(-1);
    }

    switch(c) {
    case '-': sleep(5); msg.bumper[0] = 1; break;
    case '_': sleep(5); msg.bottom_psd[0] = 1; break;/*
      case '1': msg.bumper[0] = 1; break;
      case '2': msg.bumper[1] = 1; break;
      case '3': msg.bumper[2] = 1; break;
      case '4': msg.bumper[3] = 1; break;

      case '5': msg.bottom_psd[0] = 1; break;
      case '6': msg.bottom_psd[1] = 1; break;
      case '7': msg.bottom_psd[2] = 1; break;
      case '8': msg.bottom_psd[3] = 1; break;
      case '9': msg.bottom_psd[4] = 1; break;
      case '0': msg.bottom_psd[5] = 1; break;
*/
      case 'r':
        for (unsigned int i = 0; i < msg.bumper.size(); i++)
          msg.bumper[i] = 0;
        for (unsigned int i = 0; i < msg.bottom_psd.size(); i++)
          msg.bottom_psd[i] = 0;
        break;

      case 'a': msg.touch[3] = 1; break;
      case 'm': msg.touch[3] = 0; break;
      case 'A': sleep(5); msg.touch[3] = 1; break;
      case 'M': sleep(5); msg.touch[3] = 0; break;

      case 'h': msg.touch[4] = 1; break;
      case 'l': msg.touch[4] = 0; break;

      case 'e': msg.emergency_button[0] = 1; break;
      case 'o': msg.emergency_button[0] = 0; break;

      case 'd': {
        stargazer_msgs::LocationData data;
        data.num = 1;
        data.id.push_back(rand()%10000);
        data.x.push_back( rand()%300 - 150);
        data.y.push_back( rand()%300 - 150);
        data.t.push_back((rand()%62832 - 31416)/10000.0);

        sgdata_pub.publish(data);
        break;
      }

      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9': {
        stargazer_msgs::LocationData data;
        data.num = 1;
        data.id.push_back(c - 48);
        data.x.push_back( rand()%30 - 15);
        data.y.push_back( rand()%30 - 15);
        data.t.push_back((rand()%12832 + 21416)/10000.0);

        sgdata_pub.publish(data);
        break;
      }

      case 's': {
        stargazer_msgs::LocationData data;
        data.num = 2;
        data.id.push_back(1);
        data.id.push_back(2);
        data.x.push_back(0);
        data.x.push_back(0);
        data.y.push_back(0.1);
        data.y.push_back(0.1);
        data.t.push_back(0);
        data.t.push_back(1.57075);

        sgdata_pub.publish(data);
        break;
      }

      case 'f': {
        stargazer_msgs::LocationData data;
        data.num = 3;
        data.id.push_back(288);
        data.x.push_back(100.0);
        data.y.push_back(-20.0);
        data.t.push_back(1);
        data.id.push_back(32);
        data.x.push_back(20.0);
        data.y.push_back(100.0);
        data.t.push_back(2);
        data.id.push_back(3);
        data.x.push_back(50.0);
        data.y.push_back(50.0);
        data.t.push_back(2.5);
        sgdata_pub.publish(data);
        break;
      }

      case 'D': {
        stargazer_msgs::LocationData data;
        data.num = 3;
        data.id.push_back(32);
        data.x.push_back(2.0);
        data.y.push_back(0.0);
        data.t.push_back(1.0);
        data.id.push_back(32);
        data.x.push_back(2.0);
        data.y.push_back(0.0);
        data.t.push_back(1.0);
        data.id.push_back(32);
        data.x.push_back(2.0);
        data.y.push_back(0.0);
        data.t.push_back(1.0);
        sgdata_pub.publish(data);
        break;
      }

      case 'p': {
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.seq = sg_pose_sec++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = rand()%60;
        msg.pose.pose.position.y = rand()%20;
        msg.pose.pose.orientation.z = (rand()%2)*2 - 1;
        msg.pose.pose.orientation.w = (rand()%10)/10.0;
        msg.pose.covariance[0]  = (rand()%20)/50.0;
        msg.pose.covariance[1]  = (rand()%10)/10.0;
        msg.pose.covariance[6]  = (rand()%10)/10.0;
        msg.pose.covariance[7]  = (rand()%20)/50.0;
        msg.pose.covariance[35] = (rand()%20)/50.0;
        msg.pose.covariance[14] = 99999;  // set a very large covariance on unused
        msg.pose.covariance[21] = 99999;  // dimensions (z, pitch and roll); this
        msg.pose.covariance[28] = 99999;  // is required on 6D EKF data fusion
        sgpose_pub.publish(msg);
        break;
      }

      case 'P': {
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.seq = sg_pose_sec++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = 4;
        msg.pose.pose.position.y = 1;
        msg.pose.pose.orientation.w = 1;
        msg.pose.covariance[0]  = 0.01;
        msg.pose.covariance[1]  = 0.01;
        msg.pose.covariance[6]  = 0.01;
        msg.pose.covariance[7]  = 0.01;
        msg.pose.covariance[35] = 0.01;
        msg.pose.covariance[14] = 99999;  // set a very large covariance on unused
        msg.pose.covariance[21] = 99999;  // dimensions (z, pitch and roll); this
        msg.pose.covariance[28] = 99999;  // is required on 6D EKF data fusion
        sgpose_pub.publish(msg);
        break;
      }
      case 'q':
        exit(0); quit_requested = true; break;
      default: break;
    }
  }
  tcsetattr(kfd, TCSANOW, &cooked);
}

void dumpToScreen() {
  // Dump help and current status
  puts("");
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("1, 2, 3, 4 : enable bumpers");
  puts("5, 6, 7, 8, 9, 0 : enable PSD");
  puts("r : reset all sensors");
  puts("a : switch to automatic mode");
  puts("m : switch to manual mode");
  puts("l : switch to low speed");
  puts("h : switch to high speed");
  puts("e : emergency button on");
  puts("o : emergency button off");
  puts("d : simulate stargazer data");
  puts("p : simulate stargazer pose");
  puts("");
  puts("");
  puts("Current status:");
  puts("---------------------------");
  printf("Bumpers :  ");
  for (unsigned int i = 0; i < msg.bumper.size(); i++)
    printf(" %s", msg.bumper[i]?"x":"_");
  puts("");
  printf("PSD :      ");
  for (unsigned int i = 0; i < msg.bottom_psd.size(); i++)
    printf(" %s", msg.bottom_psd[i]?"x":"_");
  puts("");
  printf("Mode :      %s\n", msg.touch[3]?"auto":"manual");
  printf("Speed :     %s\n", msg.touch[4]?"high":"low");
  printf("Emergency : %s\n", msg.emergency_button[0]?"yes":"no");
}
