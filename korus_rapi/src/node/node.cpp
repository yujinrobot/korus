/**
 * @file /korus_rapi/src/node/node.cpp
 *
 * @brief Node for korus rapi(x) communications.
 *
 * @date October 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "../../include/korus_rapi/korus_rapi.hpp"

/*****************************************************************************
** Main
*****************************************************************************/



int main(int argc, char **argv) {

	ros::init(argc,argv,"korus_rapi");

	korus_rapi::KorusRapi rapi("192.168.2.1", 5002, "ace", true );
//	korus_rapi::KorusRapi rapi("192.168.10.189", 5002, "ace", true );
	rapi.init();

	ros::spin();

	return 0;
}
