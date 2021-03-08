#!/usr/bin/env python
import rospy

remote_manipulator_ip = rospy.get_param('/communication_between_manipulators/remote_manipulator_ip')
recv_port = rospy.get_param('/communication_between_manipulators/recv_port', 34343)
send_port = rospy.get_param('/communication_between_manipulators/send_port', 34344)
