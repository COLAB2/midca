import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from MIDCA.modules._robot_world import world_repr

class RosMidca:
	
	def __init__(self, midcaObj, incomingMsgHandlers = [], outgoingMsgHandlers = [],
	dynamicPublishers = False):
		self.midca = midcaObj
		self.incomingMsgHandlers = incomingMsgHandlers
		self.outgoingMsgHandlers = outgoingMsgHandlers
		self.dynamicPublishers = False
	
	def ros_connect(self):
		rospy.init_node("MIDCA")
		for handler in self.incomingMsgHandlers:
			handler.subscriber = rospy.Subscriber(handler.topic, handler.msgType, handler.callback)
		for handler in self.outgoingMsgHandlers:
			handler.publisher = rospy.Publisher(handler.topic, handler.msgType)
	

	def send_msg(self, topic, msg):
		'''
		It is possible to have multiple handlers for a topic. Each handler, barring errors,
		will send one message. In the case of multiple handlers, the outgoing order is
		unspecified by this interface. Note also that a handler may return None, in which
		case no message will be sent. This can be used to, for example, create two handlers,
		only one of which is active depending on the msg argument. Of course, it would
		also be possible to have one handler implement both behaviors, but this might not
		be desirable.
		'''
		for handler in self.outgoingMsgHandlers:
			if handler.topic == topic:
				if handler.publisher == None:
					rospy.logerr("publisher for topic " + str(topic) " has not been \
					initialized. Msg " + str(msg) + " Will not be sent.")
				else:
					if hasattr(handler, "convert_outgoing_msg"):
						try:
							convertedMsg = handler.convert_outgoing_msg(msg)
						except:
							rospy.logerr("Message conversion failed for msg: " + str(msg) +
							". Msg will not be sent.")
							convertedMsg = None
					else:
						if not isinstance(msg, handler.msgType):
							rospy.logerr("No message conversion function specified and msg " +
							str(msg) " is of type " + str(type(msg)) + " where " + 
							str(handler.msgType) + " was expected. Msg will not be sent.")
							convertedMsg = None
						else:
							convertedMsg = msg
					if convertedMsg:
						rospy.loginfo("Midca sending msg: " + str(convertedMsg))
						handler.publisher.publish(convertedMsg)
				
	

class IncomingMsgHandler:
	
	def __init__(self, topic, msgType, callback, midcaObject = None):
		if midcaObject:
			self.mem = midcaObject.mem
		else:
			self.mem = None
			rospy.logwarn("incoming msg handler created with no pointer to MIDCA object. \
			This may not be useful unless a reference is passed elsewhere.")
		self.topic = topic
		self.msgType = msgType
		self.callback = callback
		self.publisher  = None
		self.subscriber = None

class FixedObjectLocationHandler(IncomingMsgHandler):

	'''
	class that receives Point messages, where each message indicates that the given object
	has been identified at that location. Args include the topic to listen to, object id,
	and midca object to whose memory observations will be stored. Optionally the memory
	key to be stored to can also be specified.
	'''
	
	def __init__(self, topic, objID, midcaObject, memKey = None):
		callback = lambda pointMsg: self.store_location(pointMsg)
		msgType = Point
		super(FixedObjectLocationHandler, self).__init__(topic, msgType, callback,
		midcaObject)
		self.objID = objID
		if memKey:
			self.memKey = memKey
		else:
			self.memKey = self.mem.ROS_OBJS_DETECTED
		
	def store_location(self, pointMsg):
		if not self.mem:
			rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
		self.mem.add(self.memKey, world_repr.DetectionEvent(id = self.objID, 
		loc = pointMsg))

class UtteranceHandler(IncomingMsgHandler):
	
	def __init__(self, topic, midcaObject, memKey = None):
		callback = lambda strMsg: self.store_utterance(strMsg)
		msgType = String
		super(UtteranceHandler, self).__init__(topic, msgType, callback, midcaObject)
		if memKey:
			self.memKey = memKey
		else:
			self.memKey = self.mem.ROS_WORDS_HEARD
	
	def store_utterance(self, utterance):
		if not self.mem:
			rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
		self.mem.add(self.memKey, world_repr.UtteranceEvent(utterance.data.strip()))
	
class OutgoingMsgHandler:
	
	def __init__(Self, topic, msgType):
		self.topic = topic
		self.msgType = msgType
		self.publisher = None
		self.subscriber = None
