from __future__ import print_function
from pocketsphinx import LiveSpeech, get_model_path
import time
import rospy
import os,inspect
from std_msgs.msg import String


model_dir  =  os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
model_dir = model_dir + "/TAR1190/"
model_path = get_model_path()
speech = LiveSpeech(
    	verbose=False,
    	sampling_rate=16000,
    	buffer_size=2048,
    	no_search=False,
    	full_utt=False,
    	hmm=os.path.join(model_path, model_dir + 'en-us'),
    	lm=os.path.join(model_path, model_dir + '1733.lm'),
    	dic=os.path.join(model_path, '1733.dic')
)


start = ['GET' , 'STACK' , 'PUT']


rospy.init_node('baxter_pointing_test')

pub = rospy.Publisher('cmds_received', String, queue_size=10)


#tuck_cmd

while not rospy.is_shutdown():
		
	for phrase in speech:
		phrase = str(phrase)
		check = phrase.split(" ")
		print(str(phrase))
		if not check is None:
			if check[0] in start:
				if len(check) >= 4 and len(check) < 9 :
					 	#print(str(phrase)) 
						print("Sending voice command:")
						pub.publish(str(phrase).lower())
						time.sleep(1)
						pub.publish(str(phrase).lower())
						time.sleep(1)
						
						
	
	'''
	
	#pub.publish("get the red block")
	#pub.publish("put the red block on table")
	#pub.publish("put the green block on table")
	time.sleep(4)
	print("stack the blue block on the red block")
	#pub.publish("stack the blue block on the red block")
	pub.publish("stack the green block on the blue block")
	time.sleep(1)
	pub.publish("stack the green block on the blue block")
	#pub.publish("stack the blue block on the red block")
	print("Sending voice command to MIDCA:")
	#print("Sending voice command to MIDCA:")
	#pub.publish("stack the blue block on the red block")
	#print("stack the blue block on the red block")
	#pub.publish("stack the green block on the blue block")
	time.sleep(2)
	time.sleep(50000)
	
	'''
   
# n = 0
# while n < 20:
# 	n += 1d
# 	time.sleep(2)
# 	p = points[n % 4]
# #	p = points[1]
# 	print("Sending point command:", p)
# 	pub.publish(p)

