#!/usr/bin/env python
import roslib; roslib.load_manifest('korus_smach')
import rospy
import smach_ros
from korus_smach.state_machines import head_calibration


'''
 Head calibration
'''
def main():
    rospy.init_node('head_calibration')
    
    sm_head_calibration = head_calibration.createSM()
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('introspection_server', sm_head_calibration, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    sm_head_calibration.execute()
    
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

