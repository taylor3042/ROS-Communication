#!/usr/bin/env python3
# ~/catkin_ws/src/bluetooth_interface/src/bluetooth_interface_node.py

import rospy
from std_msgs.msg import Float64
import bluetooth

rospy.init_node('bluetooth_interface_node')
distance_pub = rospy.Publisher('target_distance', Float64, queue_size=10)

# Assuming you have a function to handle Bluetooth connection

server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 1
server_sock.bind(("", port))
server_sock.listen(1)
client_sock, address = server_sock.accept()
print("Accepted connection from", address)


while True:

    recvdata = client_sock.recv(1024)
    print ("Received \"%s\" through Bluetooth" % recvdata)
    # use a try except to make sure that it's a number, 
    # if not it will just print to the console that this isn't what it's looking for.
    try:
        target_distance = float(recvdata)
        distance_pub.publish(target_distance)
    except:
        print("not a number")

