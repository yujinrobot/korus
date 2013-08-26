/**
 * @file /korus_rapi/src/lib/korus_rapi.cpp
 *
 * @brief Implementation for the korus rapi mainframe tcp/ip client.
 *
 * @date 19/10/2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include "../../include/korus_rapi/korus_rapi.hpp"
#include <ecl/threads.hpp>
#include "../../include/korus_rapi/korus_rapi_protocol.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace korus_rapi {

/*****************************************************************************
** Using
*****************************************************************************/
/**
 * @note
 * sequence ;
 * - making slot
 * - making client
 * - you should keep this sequence; do not touch.
 * @param server
 * @param port
 * @param name
 * @param activateClient
 * @return
 */
KorusRapi::KorusRapi( const std::string & server, const int & port, const std::string & name, bool activateClient )
	: name("rapi_client"),
	data_slot( &KorusRapi::receiver,* this, name ),
	init_slot( & KorusRapi::notify, *this, name + std::string("_init") ),
	client( server, port, name, true ),
	slot_arm_response( &KorusRapi::armResponseCB,* this, "sig_arm_command_interface"),
	slot_head_response( &KorusRapi::headResponseCB,* this, "sig_head_command_interface"),
	slot_gripper_response( &KorusRapi::gripperResponseCB,* this, "sig_gripper_command_interface")
{
}


void KorusRapi::init()
{
	// in here, we need rosparam configuration for the server (aka see joint_trajectory_controller init())
	// basically it will be the same options as we have in parser.cpp here.

	 // run
	ecl::Thread updater( &KorusRapi::update, *this );
}


/**
 * slot callback, when rapiBase get data set, it will call below funciton
 *
 * @param packet : this is basepacket which has header information and payload
 */
void KorusRapi::receiver( const rapi_comms::base_packet_t & packet )
{
	// show packet
	rapi_comms::rapiBase::showMePacket( packet );

	// store
	lock_read.lock();
	packet_que.push_back( packet );
//	printf("==size of packet_que is %d ==\n", packet_que.size() );
	lock_read.unlock();
}

/**
 * main thread to update everythings
 */
void KorusRapi::update()
{
	while( ros::ok() )
	{
		// we need event later
		usleep(10000);

		// do something
		doSomething();
	}
}


/**
 * whenever distributed engine connects to mainframe,
 * they have to send the packet to notify mainframe
 * @todo; have to ask to youngil choi
 */
void KorusRapi::notify()
{
	std::cout << "send the notification codes to mainframe " << std::endl;
	rapi_comms::base_packet_t packet;

	// payload first,
	unsigned int result_code(0);	// success
	unsigned short int app_id(9);

	packet.addPayload( result_code );
	packet.addPayload( app_id );

	// header
	packet.header.Init( packet.payload.size(), 0, 0x0121, app_id, 0, 4  );

	client.setFeedbackToMainframe( packet );
}

/*****************************************************************************
** I have assumed that we have four packet que whose maximum length is one.
*****************************************************************************/

void KorusRapi::armResponseCB( int response )
{
	std::cout << "armResponseCB[" << response << "]" << std::endl;
	if( packet_que_arm.empty() )
	{
		std::cout << "zeor packet que " << std::endl;
		return;
	}

	// send the response and pop que
	lock_read.lock();
	rapi_comms::base_packet_t packet = packet_que_arm[0];
	packet_que_arm.pop_front();
	lock_read.unlock();

	packet.payload.clear();
	packet.addPayload( response );
	//@todo; put the error data
	if( packet.header.wCmdType == 0x1034 )
	{
		unsigned int error_data(0x01);
		packet.addPayload( error_data );
	}

	client.setResponse( packet );
}
void KorusRapi::headResponseCB( int response )
{
	std::cout << "headResponseCB[" << response << "]" << std::endl;
	if( packet_que_head.empty() )
	{
		std::cout << "zeor packet que " << std::endl;
		return;
	}

	// send the response and pop que
	lock_read.lock();
	rapi_comms::base_packet_t packet = packet_que_head[0];
	packet_que_head.pop_front();
	lock_read.unlock();

	packet.payload.clear();
	packet.addPayload( response );
	client.setResponse( packet );
}

void KorusRapi::gripperResponseCB( int response )
{
	std::cout << "gripperResponseCB[" << response << "]" << std::endl;
	if( packet_que_gripper.empty() )
	{
		std::cout << "zeor packet que " << std::endl;
		return;
	}

	// send the response and pop que
	lock_read.lock();
	rapi_comms::base_packet_t packet = packet_que_gripper[0];
	packet_que_gripper.pop_front();
	lock_read.unlock();

	packet.payload.clear();
	packet.addPayload( response );
	client.setResponse( packet );
}

void KorusRapi::doSomething()
{
	bool valid_packet;
	unsigned int indexer(0);
	while( !packet_que.empty() )
	{
		valid_packet = true;

		lock_read.lock();
		rapi_comms::base_packet_t packet = packet_que[0];
		packet_que.pop_front();
		lock_read.unlock();

		// should i ack
		if( packet.ack() )
		{
			client.setAck( packet.header );
		}
		indexer = 1;	// except ack byte

		// verbose
		std::cout << std::endl;
		std::cout << "KorusRapi[";
		printf("%04x] ", packet.header.wCmdType );

		// first data of payload is for ack

		// according to the cmd_type of jobs
		switch( packet.header.wCmdType )
		{
			case rssp_joint_control_of_arm:
			{
				int joint_id(-1);
				unsigned int control_mode  	= rapi_comms::buildValue<unsigned int>()( packet.payload, indexer );
				unsigned int joint_id_field			= rapi_comms::buildValue<unsigned int>()( packet.payload, indexer );
				double target_angle  				= rapi_comms::buildValue<double>()		    ( packet.payload, indexer );
				//unsigned int control_method  	= rapi_comms::buildValue<unsigned int>()( packet.payload, indexer );
				//@todo; clean up garbages
				for( int i=0; i<7; i++ )
				{
					if( joint_id_field & (1<<i) )
					{
						joint_id = i;
						break;
					}
				}
				std::cout << "control Information[joint|mode|angle|method] : ";
				printf("%02x|%d|%+1.2f|%d ", joint_id_field, control_mode, target_angle, control_mode );

				unsigned int faluty_joint(0);
				if( joint_id < 0 || joint_id >= 7 )
				{
					// invalid id
					packet.payload.clear();
					packet.addPayload( -1 );
					packet.addPayload( faluty_joint );
					client.setResponse( packet );
				}
				else
				{
					// valid joint id
					std::vector<double> joint_angles(7,0.0);
					for( int i=0; i<7; i++ ) joint_angles[i] = arm.joint_angles[i];
					joint_angles[joint_id] = target_angle;

					std::vector<double> joint_angles_cmd;
					for( unsigned int i=0; i<5; i++)
					{
						joint_angles_cmd.push_back(joint_angles[i]);
					}

					addPacketArm( packet );
					arm.sendTrajectory( joint_angles_cmd, control_mode );
				}

				break;
			}

			case rssp_get_joint_value_of_arm:
			{
				int joint_id(-1);
				unsigned int joint_id_field				= rapi_comms::buildValue<unsigned int>()( packet.payload, indexer );
				unsigned int control_mode  	= rapi_comms::buildValue<unsigned int>()( packet.payload, indexer );

				for( int i=0; i<7; i++ )
				{
					if( joint_id_field & (1<<i) )
					{
						joint_id = i;
						break;
					}
				}
				joint_id = joint_id_field;
				std::cout<<"get_joint value: "<<joint_id_field<<" : "<<joint_id <<std::cout;

				if( joint_id < 0 || joint_id >= 7 )
				{
					// invalid id
					packet.payload.clear();
					packet.addPayload( (unsigned int)-1 );
					packet.addPayload( control_mode );
					packet.addPayload( (unsigned int)0 );
					packet.addPayload( (double)0.0 );
					client.setResponse( packet );
				}
				else
				{
					packet.payload.clear();
					packet.addPayload( (unsigned int)0 );
					packet.addPayload( control_mode );
					packet.addPayload( (unsigned int)joint_id_field );
					packet.addPayload( (double)arm.joint_angles[joint_id] );
					client.setResponse( packet );
					std::cout << "control Information[error|joint|mode|angle] : ";
					printf("0|%02x|%d|%+1.2f ", joint_id_field, control_mode, arm.joint_angles[joint_id] );
				}

				break;
			}

			case rssp_set_eef_of_pose:
			{
				// pose information
				std::vector<double>pose_command(7,0.0);
				std::cout << "Target Pose: ";
				for( int i=0; i<7; i++ )
				{
					pose_command[i] = rapi_comms::buildValue<double>()( packet.payload, indexer );
					printf("[%+1.2f]", pose_command[i] );
				}
				std::cout << std::endl;

				addPacketArm( packet );
				arm.solveTargetPose( pose_command );
				break;
			}

			case rssp_all_joint_control_of_arm:
			{
				std::vector<double> joint_angles(7,0.0);
				unsigned int mode( rapi_comms::buildValue<unsigned int>()( packet.payload, indexer) );
				unsigned int num_of_joint( rapi_comms::buildValue<unsigned int>()( packet.payload, indexer) );
				std::cout << "Joint angles: ";
				for( unsigned int i=0; i<num_of_joint; i++ )
				{
					joint_angles[i] = rapi_comms::buildValue<double>()( packet.payload, indexer );
					printf("[%+1.2f]", joint_angles[i] );
				}
				std::cout << std::endl;
				unsigned int method( rapi_comms::buildValue<unsigned int>()( packet.payload, indexer) );
				std::cout << "attributes[Mode/Num/Method]" << mode << "/" << num_of_joint << "/" << method << std::endl;

				//@todo; specified the mode and method
				//@todo; specified the error code
				std::vector<double> joint_angles_cmd;
				for( unsigned int i=0; i<5; i++)
				{
					joint_angles_cmd.push_back(joint_angles[i]);
				}

				addPacketArm( packet );
				arm.sendTrajectory( joint_angles_cmd, mode );
				break;
			}

			case rssp_get_status_of_arm:
			{
				//@todo; assumed that there are not problem
				unsigned int joint_field(0);
				packet.payload.clear();
				packet.addPayload( (unsigned int)0 );
				packet.addPayload( (unsigned int)joint_field );
				client.setResponse( packet );
				std::cout << "jointValue[error|problem] : [0|0] ";
				break;
			}

			case rssp_set_pose_of_arm:
			{
				unsigned int pose_id( rapi_comms::buildValue<unsigned int>()( packet.payload, indexer) );
				std::cout << "target pose id " << pose_id;

				//@todo; now this function is not available
				unsigned int error_code(-1);
				packet.payload.clear();
				packet.addPayload( (unsigned int)error_code );
				client.setResponse( packet );

				break;
			}

/*			case rssp_set_status_gripper:
			{
				double target_angle(0.0);
				unsigned int gripper_status( rapi_comms::buildValue<unsigned int>()( packet.payload, indexer) );
				std::cout << "desired gripper status: " << gripper_status;
				switch( gripper_status )
				{
				case 0:	// close
					target_angle = 0.2;
					addPacketGripper( packet );
					gripper.send( target_angle );
					break;
				case 1: // open
					target_angle = 1.0;
					addPacketGripper( packet );
					gripper.send( target_angle );
					break;
				case 2: // ready
					target_angle = 0.5;
					addPacketGripper( packet );
					gripper.send( target_angle );
					break;
				default:
					unsigned int error_code(-1);
					packet.payload.clear();
					packet.addPayload( (unsigned int)error_code );
					client.setResponse( packet );
					break;
				}

				break;
			}
*/

			case rssp_set_status_gripper:
			{
				double target_angle(0.0);
				int gripper_status( rapi_comms::buildValue<int>()( packet.payload, indexer) );
				std::cout << "desired gripper status: " << gripper_status;

				target_angle = gripper_status * 3.141592/180.0;
				addPacketGripper( packet );
				gripper.send( target_angle );

				break;
			}


			case rssp_get_status_gripper:
			{
				unsigned int error_code(0);
				unsigned int status(0);

				packet.payload.clear();
				packet.addPayload( (unsigned int)error_code );

//				if( gripper.joint_angle <= 0.15 )
//				{
//					status = 0;
//				}
//				else if( gripper.joint_angle <= 0.55)
//				{
//					status = 2;
//				}
//				else
//				{
//					status = 1;
//				}

				status = (unsigned int) ( gripper.joint_angle * 180.0 / 3.141592) ;
				packet.addPayload( (unsigned int)status );
				client.setResponse( packet );

				break;
			}


			case rssp_get_all_joint_value_of_arm:
			{
				// receive the data from the mainframe
				unsigned int dmode( rapi_comms::buildValue<unsigned int>()( packet.payload, indexer) );

				// make packet to send the data
				unsigned int error_code(0);		// error code why?
				unsigned int mode(dmode);	// position mode; position(0), velocity(1), current(2)
				unsigned int num_of_joint(7);	// seven joint

				packet.payload.clear();
				packet.addPayload( error_code );
				packet.addPayload( mode );
				packet.addPayload( num_of_joint );
				std::cout << "Joint angles: ";
				for( int i=0; i<7; i++ )
				{
					packet.addPayload( arm.joint_angles[i] );
					printf("[%+1.2f]", arm.joint_angles[i] );
				}
				std::cout << std::endl;
				client.setResponse( packet );
				break;
			}

			case rssp_set_angle_of_camera:
			{
				std::vector<double>target_angles(2, 0.0);
				target_angles[0] = rapi_comms::buildValue<double>()( packet.payload, indexer );
				target_angles[1] = rapi_comms::buildValue<double>()( packet.payload, indexer );
				addPacketHead( packet );
				head.sendTrajectory( target_angles );
				std::cout << "Joint angles: " << target_angles[0] << ", " << target_angles[1] << std::endl;
				break;
			}

			case rssp_get_angle_of_camera:
			{
				// build the packet
				unsigned int result_code(0);
				packet.payload.clear();
				packet.addPayload( result_code );
				packet.addPayload( head.joint_angles[0] );
				packet.addPayload( head.joint_angles[1] );
				client.setResponse( packet );
				std::cout << "Joint angles: " << head.joint_angles[0] << ", " << head.joint_angles[1] << std::endl;
				break;
			}

			default:
				valid_packet = false;
				break;
		}

		std::cout << std::endl;

	}// while
}

} // namespace korus_rapi
