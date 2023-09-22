#!/usr/bin/env python

import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1" # Topside (local) IP
UDP_PORT = 9999        # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP

sock.bind((UDP_IP, UDP_PORT))

def datos_publisher():
    pub = rospy.Publisher('dvl_out', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode().strip()
        #.strip()
        #print (dvl_datos)
        #a = 3
        rospy.loginfo(dvl_datos)
        pub.publish(dvl_datos)
        rate.sleep()
   
if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass
