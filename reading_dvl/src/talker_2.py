import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999         # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet, UDP
sock.bind((UDP_IP, UDP_PORT))

def datos_publisher():
    pub = rospy.Publisher('dvl_out', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode()

        dvpdl_start_index = dvl_datos.find('$DVPDL') + len('$DVPDL,')
        dvpdl_end_index = dvl_datos.find(',', dvpdl_start_index)
        element_8 = dvl_datos[dvpdl_start_index:dvpdl_end_index]

        dvpdl_start_index = dvpdl_end_index + 1
        dvpdl_end_index = dvl_datos.find('*', dvpdl_start_index)
        element_9 = dvl_datos[dvpdl_start_index:dvpdl_end_index]

        dvext_start_index = dvl_datos.find('$DVEXT') + len('$DVEXT,')
        dvext_end_index = dvl_datos.find(',', dvext_start_index)
        element_10 = dvl_datos[dvext_start_index:dvext_end_index]

        dvext_start_index = dvext_end_index + 1
        dvext_end_index = dvl_datos.find(',', dvext_start_index)
        element_11 = dvl_datos[dvext_start_index:dvext_end_index]

        rospy.loginfo("Element 8: {}".format(element_8))
        rospy.loginfo("Element 9: {}".format(element_9))
        rospy.loginfo("Element 10: {}".format(element_10))
        rospy.loginfo("Element 11: {}".format(element_11))

        pub.publish(dvl_datos)
        rate.sleep()

if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass




#///////////////////////////////////////////////////////////////////////////

import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999         # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))

def datos_publisher():
    pub = rospy.Publisher('dvl_out', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode()

        dvpdl_start_index = dvl_datos.find('$DVPDL') + len('$DVPDL,')
        dvpdl_end_index = dvl_datos.find(',', dvpdl_start_index)
        element_8_9 = dvl_datos[dvpdl_start_index:dvpdl_end_index]

        dvext_start_index = dvl_datos.find('$DVEXT') + len('$DVEXT,')
        dvext_end_index = dvl_datos.find(',', dvext_start_index)
        element_10_11 = dvl_datos[dvext_start_index:dvext_end_index]

        rospy.loginfo(f"Element 8 and 9: {element_8_9}")
        rospy.loginfo(f"Element 10 and 11: {element_10_11}")

        pub.publish(dvl_datos)
        rate.sleep()

if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass



#///////////////////////////////////////////////////////////////////////////
import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999  # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet, UDP
sock.bind((UDP_IP, UDP_PORT))

def datos_publisher():
    pub = rospy.Publisher('dvl_out', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode().splitlines()

        dvext_data = dvl_datos[0].split(',')
        element_10 = dvext_data[10]
        element_11 = dvext_data[11]

        dvpdl_data = dvl_datos[1].split(',')
        element_6 = dvpdl_data[6]
        element_7 = dvpdl_data[7]

        rospy.loginfo("Element 10: %s", element_10)
        rospy.loginfo("Element 11: %s", element_11)
        rospy.loginfo("Element 6: %s", element_6)
        rospy.loginfo("Element 7: %s", element_7)

        pub_data = f"Element 10: {element_10}, Element 11: {element_11}, Element 6: {element_6}, Element 7: {element_7}"
        pub.publish(pub_data)

        rate.sleep()

if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass


#///////////////////////////////////////////////////////////////////////////

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


#///////////////////////////////////////////////////////////////////////////

import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999         # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))

def datos_publisher():
    pub = rospy.Publisher('dvl_out', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode()  # Convert bytes to string
        rospy.loginfo(dvl_datos)
        
        # Encontar $DVEXT y extraer los elementos 10 y 11
        if "$DVEXT" in dvl_datos:
            start_index = dvl_datos.find("$DVEXT") + len("$DVEXT")
            end_index = dvl_datos.find(",", start_index)
            element_10 = dvl_datos[start_index:end_index]
            start_index = end_index + 1
            end_index = dvl_datos.find(",", start_index)
            element_11 = dvl_datos[start_index:end_index]
            rospy.loginfo("Element 10: {}, Element 11: {}".format(element_10, element_11))
            pub.publish("Element 10: {}, Element 11: {}".format(element_10, element_11))

        
        # Encontrar $DVPDL y extraer los elementos 6 y 7
        if "$DVPDL" in dvl_datos:
            start_index = dvl_datos.find("$DVPDL") + len("$DVPDL")
            end_index = dvl_datos.find(",", start_index)
            element_6 = dvl_datos[start_index:end_index]
            start_index = end_index + 6
            end_index = dvl_datos.find(",", start_index)
            element_7 = dvl_datos[start_index:end_index]
            rospy.loginfo("Element 6: {}, Element 7: {}".format(element_6, element_7))
            pub.publish("Element 6: {}, Element 7: {}".format(element_6, element_7))

        rate.sleep()

if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass





#///////////////////////////////////////////////////////////////////////////

import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999  # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))

def datos_publisher():
    pub_dvext = rospy.Publisher('dvext_data', String, queue_size=20)
    pub_dvpdl = rospy.Publisher('dvpdl_data', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode() # Convert bytes to string
        dvl_lines = dvl_datos.strip().split('\n')
        rospy.loginfo(dvl_datos)
        
        for line in dvl_lines:
            if line.startswith('$DVEXT'):
                elements = line.split(',')
                if len(elements) >= 12:
                    element_10 = elements[10]
                    element_11 = elements[11]
                    rospy.loginfo("Element 10: {}, Element 11: {}".format(element_10, element_11))
                    pub_dvext.publish("Element 10: {}, Element 11: {}".format(element_10, element_11))

            if line.startswith('$DVPDL'):
                elements = line.split(',')
                if len(elements) >= 8:
                    element_6 = elements[6]
                    element_7 = elements[7]
                    rospy.loginfo("Element 6: {}, Element 7: {}".format(element_6, element_7))
                    pub_dvpdl.publish("Element 6: {}, Element 7: {}".format(element_6, element_7))

        rate.sleep()

if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass


#///////////////////////////////////////////////////////////////////////////

import rospy
import socket
from std_msgs.msg import String

UDP_IP = "192.168.2.1"  # Topside (local) IP
UDP_PORT = 9999  # Topside (local) port to listen on

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))

def datos_publisher():
    pub_dvext = rospy.Publisher('dvext_data', String, queue_size=20)
    pub_dvpdl = rospy.Publisher('dvpdl_data', String, queue_size=20)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        dvl_datos = data.decode() # Convert bytes to string
        dvl_lines = dvl_datos.strip().split('\n')
        #rospy.loginfo(dvl_lines)
        
        for line in dvl_lines:
            if line.startswith('$DVEXT'):
                elements = line.split(',')
                if len(elements) >= 12:
                    element_10 = elements[10]
                    element_11 = elements[11]
                    rospy.loginfo("Element 10: {}, Element 11: {}".format(element_10, element_11))
                    pub_dvext.publish("Element 10: {}, Element 11: {}".format(element_10, element_11))

            if line.startswith('$DVPDL'):
                elements = line.split(',')
                if len(elements) >= 8:
                    element_6 = elements[6]
                    element_7 = elements[7]
                    rospy.loginfo("Element 6: {}, Element 7: {}".format(element_6, element_7))
                    pub_dvpdl.publish("Element 6: {}, Element 7: {}".format(element_6, element_7))

        rate.sleep()

if __name__ == '__main__':
    try:
        datos_publisher()
    except rospy.ROSInterruptException:
        pass