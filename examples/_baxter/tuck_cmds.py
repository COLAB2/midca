from __future__ import print_function
import time
import rospy
from std_msgs.msg import String

rospy.init_node('baxter_pointing_test')

pub = rospy.Publisher('tuck_cmd', String, queue_size=10)
#tuck_cmd
while not rospy.is_shutdown():
	print("Sending voice command:")
	pub.publish("tuck the arms")
	time.sleep(2)
   
# n = 0
# while n < 20:
# 	n += 1
# 	time.sleep(2)
# 	p = points[n % 4]
# #	p = points[1]
# 	print("Sending point command:", p)
# 	pub.publish(p)
