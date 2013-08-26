/**
 * @file /korus_rapi/include/korus_rapi/korus_rapi.hpp
 *
 * @brief Interface for rapi communications.
 *
 * @date 19/10/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KORUS_RAPI_HPP_
#define KORUS_RAPI_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <rapi_comms/rapi_comms.hpp>
#include "../../include/korus_rapi/arm_interface.hpp"
#include "../../include/korus_rapi/head_interface.hpp"
#include "../../include/korus_rapi/gripper_interface.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace korus_rapi {

/*****************************************************************************
** Using
*****************************************************************************/


class KorusRapi
{
public:
	KorusRapi( const std::string & server, const int & port, const std::string & name, bool activateClient=true );
	virtual ~KorusRapi() {}
	void init();


private:
	ecl::Mutex lock_read;
	std::deque< rapi_comms::base_packet_t > packet_que;
	std::string name;
	ecl::Slot< const rapi_comms::base_packet_t & > data_slot;
	ecl::Slot<> init_slot;
	rapi_comms::rapiBase client;


	/**
	 * slot callback, when rapiBase get data set, it will call below funciton
	 *
	 * @param packet : this is basepacket which has header information and payload
	 */
	void receiver( const rapi_comms::base_packet_t & packet );

	/**
	 * main thread to update everythings
	 */
	void update();

	/**
	 * whenever distributed engine connects to mainframe,
	 * they have to send the packet to notify mainframe
	 */
	void notify();


	void doSomething();

private:
	ArmInterface arm;
	HeadInterface head;
	GripperInterface gripper;

	ecl::Slot<int> slot_arm_response;
	ecl::Slot<int> slot_head_response;
	ecl::Slot<int> slot_gripper_response;

	void armResponseCB( int response );
	void headResponseCB( int response );
	void gripperResponseCB( int response );

	std::deque< rapi_comms::base_packet_t > packet_que_arm;
	std::deque< rapi_comms::base_packet_t > packet_que_head;
	std::deque< rapi_comms::base_packet_t > packet_que_gripper;

	void addPacketArm( const rapi_comms::base_packet_t & packet ) { lock_read.lock(); packet_que_arm.push_back( packet ); lock_read.unlock(); }
	void addPacketHead( const rapi_comms::base_packet_t & packet ) { lock_read.lock(); packet_que_head.push_back( packet ); lock_read.unlock(); }
	void addPacketGripper( const rapi_comms::base_packet_t & packet ) { lock_read.lock(); packet_que_gripper.push_back( packet ); lock_read.unlock(); }
};


} // namespace korus_rapi

#endif /* KORUS_RAPI_HPP_ */
