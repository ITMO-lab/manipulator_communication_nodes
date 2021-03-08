#!/usr/bin/env python
import socket
import rospy
from std_msgs.msg import String

remote_manipulator_ip = rospy.get_param('/communication_between_manipulators/remote_manipulator_ip')
recv_port = rospy.get_param('/communication_between_manipulators/recv_port', 34343)
send_port = rospy.get_param('/communication_between_manipulators/send_port', 34344)

recv_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_udp_socket.bind(('localhost', recv_port,))
recv_udp_socket.settimeout(10)

send_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_callback(data):
    send_udp_socket.sendto(data.data.encode(), (remote_manipulator_ip, send_port))


def manipulator_to_manipulator():
    send = rospy.Subscriber('/communication_between_manipulators/send', String, send_callback)
    recv = rospy.Publisher('/communication_between_manipulators/recv', String, queue_size=100)
    rospy.init_node('manipulator_to_manipulator', anonymous=True)
    while not rospy.is_shutdown():
        try:
            data, addr = recv_udp_socket.recvfrom(1024)
            recv.publish(String(data=data.decode()))
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logerr(e)
            continue


if __name__ == '__main__':
    try:
        manipulator_to_manipulator()
    except rospy.ROSInterruptException:
        pass
