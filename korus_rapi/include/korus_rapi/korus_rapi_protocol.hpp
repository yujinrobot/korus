/*
 * korus_rapi_protocol.hpp
 *
 *  Created on: 02/11/2010
 *      Author: jakan2
 */

#ifndef KORUS_RAPI_PROTOCOL_HPP_
#define KORUS_RAPI_PROTOCOL_HPP_

namespace korus_rapi {

const unsigned int rssp_joint_control_of_arm				=0x1033;
const unsigned int rssp_all_joint_control_of_arm			=0x1034;
const unsigned int rssp_get_joint_value_of_arm			=0x1035;
const unsigned int rssp_get_all_joint_value_of_arm 		=0x1036;
const unsigned int rssp_get_status_of_arm 					=0x1037;
const unsigned int rssp_set_pose_of_arm 					=0x1038;
const unsigned int rssp_set_eef_of_pose						=0x1039;


const unsigned int rssp_set_status_gripper					= 0x0071;
const unsigned int rssp_get_status_gripper					= 0x0072;

const unsigned int rssp_set_tilt_angle_of_display			=0x0211;		// head
const unsigned int rssp_get_tilt_angle_of_display			=0x0212;		// head

const unsigned int rssp_set_tilt_of_stereo_cam			=0x0081;
const unsigned int rssp_get_tilt_of_stereo_cam			=0x0082;

const unsigned int rssp_set_angle_of_camera				=0x0083;
const unsigned int rssp_get_angle_of_camera				=0x0084;


};// namespace korus_rapi


#endif /* KORUS_RAPI_PROTOCOL_HPP_ */
