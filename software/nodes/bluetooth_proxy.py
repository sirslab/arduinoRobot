#!/usr/bin/env python

import serial
import time
from mob_robots.msg import VectorStamped
import rospy

com = serial.Serial('/dev/rfcomm0',baudrate=9600, timeout = 3)

def callback_diff_drive(data):
	cmd = str(int(data.data[0]*100))+ "," +str(int(data.data[1]*100)) + ',\n'
	print("[BT]" + cmd)
	com.write(cmd)
	if len(data.data) == 3:
		rospy.sleep(data.data[2])
		cmd = str(int(0)) + "," + str(int(0)) + ',\n'
		print("[BT]"+cmd)
		com.write(cmd)

def main():
	rospy.init_node('bluetooth_proxy')
	
	rospy.Subscriber('commands', VectorStamped, callback_diff_drive)
	print("subscribed")
	while not rospy.is_shutdown():
		rospy.spin()
	com.close()

if __name__ == '__main__':
    main();
   
