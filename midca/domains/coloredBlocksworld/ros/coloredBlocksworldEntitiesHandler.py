# https://github.com/COLAB2/midca/blob/c3e4cc9fab5683f44435816d90a7a2eb591c4ce4/midca/rosrun.py

class ../domains/coloredBlocksworld/ros/coloredBlocksworldEntitiesHandler(IncomingMsgHandler):

    '''
    class that receives Point messages, where each message indicates that the given object
    has been identified at that location. Args include the topic to listen to, object id,
    and midca object to whose memory observations will be stored. Optionally the memory
    key to be stored to can also be specified.
    '''
    
    def __init__(self, topic, midcaObject, memKey = None, history = None):
        callback = lambda strMsg: self.store(strMsg)
        msgType = String
		self.left = None
        super(../domains/coloredBlocksworld/ros/coloredBlocksworldEntitiesHandler, self).__init__(topic, msgType, callback, midcaObject)
        #self.objID = objID
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ROS_OBJS_DETECTED
		if history:
			self.history = history
		else:
			self.history = self.mem.STATE_HISTORY
        
    def store(self, data):
        if not self.mem:
            rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
		
		#TODO: Code should handle detecting the following relations (predicates):
        #block: ['on', 'clear']
