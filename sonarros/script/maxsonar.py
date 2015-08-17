#!/usr/bin/env python

import serial
import rospy
import select
from geometry_msgs.msg import Vector3Stamped

if __name__=="__main__":
	rospy.init_node('maxsonar')

	devname  = rospy.get_param("~devname", "/dev/ttyUSB0");
	baudrate = rospy.get_param("~baudrate", 9600);

	pub = rospy.Publisher("~height", Vector3Stamped, queue_size=10)
	
	try:
		ser = serial.Serial(port=devname, baudrate=9600, timeout=10)
		rospy.loginfo("Port <%s> open successfully."%devname)
	except Exception,e:
		raise e


	try:
		while not rospy.is_shutdown():
			buf = ser.read(1)
			if buf[0]=='R':
				ser.read(4)
				break

		while not rospy.is_shutdown():
			buf = ser.read(5)
			if buf[0]=='R':
				height = float(int(buf[1:4]));
				msg = Vector3Stamped();
				msg.header.stamp = rospy.Time.now();
				msg.header.frame_id = "sonar"
				msg.vector.z = height/100.0
				pub.publish(msg)
			else:
				while not rospy.is_shutdown():
					buf = ser.read(1)
					if buf[0]=='\n':
						break
	except serial.SerialTimeoutException,e:
		rospy.logerr("MaxSonar read timeout!")
	except select.error:
		rospy.logerr("Ctrl-C singal, exit...")
	finally:
		ser.close()

