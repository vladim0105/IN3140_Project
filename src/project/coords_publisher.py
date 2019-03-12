#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import Point

def sendData():
	publisher = rospy.Publisher("/coords", Point, queue_size=10)
	rospy.init_node("coords_publisher", anonymous=True)
	rate = rospy.Rate(10)
	pos = Point()
	pos.x = 1
	pos.y = 2
	pos.z = 3

	while not rospy.is_shutdown():
		rospy.loginfo(pos)
		publisher.publish(pos)
		rate.sleep()

if __name__ == "__main__":
	try: 
		sendData()
	except rospy.ROSInterruptException:
		pass
	
	
	
