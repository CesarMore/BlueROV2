#!/usr/bin/env python

import rospy
import libsbp
from libsbp import sbp_msg
from libsbp.system import sbp_state, sbp_startup, sbp_piksi, sbp_gps
from sensor_msgs.msg import NavSatFix

def callback(msg):
    # Extract GPS data from the SBP message
    latitude = msg.lat
    longitude = msg.lon
    altitude = msg.height

    # Create a NavSatFix message
    navsat_fix = NavSatFix()
    navsat_fix.header.stamp = rospy.Time.now()
    navsat_fix.header.frame_id = 'gps'
    navsat_fix.latitude = latitude
    navsat_fix.longitude = longitude
    navsat_fix.altitude = altitude

    # Publish the NavSatFix message
    pub.publish(navsat_fix)

def swift_node():
    rospy.init_node('swift_navigation_node', anonymous=True)
    rospy.Subscriber('/sbp/pos_llh', sbp_gps.MsgPosLLH, callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
    swift_node()

##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#!/usr/bin/env python
import rospy
import socket
import time
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999  # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))

# Directory path for saving the data
directory_path = "/home/cesar/Documents/MATLAB/TRABJO DE TESIS/Datos"

def get_timestamp():
    return time.strftime("%Y%m%d-%H%M%S")

def create_file_path():
    timestamp = get_timestamp()
    file_name = "data_" + timestamp + ".txt"
    file_path = directory_path + "/" + file_name
    return file_path

def save_data_to_file(data):
    file_path = create_file_path()
    with open(file_path, 'w') as file:
        file.write(data + "\n")

def datos_publisher():
    pub_dvext = rospy.Publisher('dvext_data', String, queue_size=20)
    pub_dvpdl = rospy.Publisher('dvpdl_data', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode() # Convert bytes to string
        dvl_lines = dvl_datos.strip().split('\n')
        #rospy.loginfo(dvl_datos)
        
        for line in dvl_lines:
            if line.startswith('$DVEXT'):
                elements = line.split(',')
                if len(elements) >= 12:
                    element_10 = elements[10]
                    element_11 = elements[11]
                    rospy.loginfo("{},{}".format(element_10, element_11))
                    pub_dvext.publish("{}, {}".format(element_10, element_11))
                    save_data_to_file("{},{}".format(element_10, element_11))

            if line.startswith('$DVPDL'):
                elements = line.split(',')
                if len(elements) >= 8:
                    element_6 = elements[6]
                    element_7 = elements[7]
                    rospy.loginfo("{},{}".format(element_6, element_7))
                    pub_dvpdl.publish("{},{}".format(element_6, element_7))
                    save_data_to_file("{},{}".format(element_6, element_7))

        rate.sleep()

if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass






##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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







