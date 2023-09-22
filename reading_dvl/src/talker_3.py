#!/usr/bin/env python

import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999         # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP, UDP_PORT))

rospy.init_node('udp_publisher')  # Initialize the ROS node
pub = rospy.Publisher('datos_dvl75', String, queue_size=1)  # Create a ROS publisher

while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
    
    message = data.decode()
    
    pub.publish(message)  # Publish the message on the 'udp_data' topic
    
    rospy.loginfo(message)
