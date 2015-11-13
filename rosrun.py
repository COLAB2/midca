try:
        import rospy
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import String
        from MIDCA.modules._robot_world import world_repr
        from MIDCA import midcatime
except:
		pass
        #print "Unable to import ROS packages - you may not have ROS installed. This will only affect MIDCA/robot interactions. If you are not using MIDCA with a robot, please ignore this message."

FEEDBACK_TOPIC = "midcaFeedback"

rosMidca = None
nextID = 0

def send_msg(topic, msg):
    if not rosMidca:
        raise Exception("RosMidca object has not been initialized and no messages can be \
        sent")
    return rosMidca.send_msg(topic, msg)

def next_id():
    global nextID
    id = nextID
    nextID += 1
    return id

def dict_as_msg(d):
    s = ""
    for key, value in d.items():
        s += str(key) + ": " + str(value) + " | "
    if s:
        s = s[:-3] #remove last ' | '
    return String(data = s)

def msg_as_dict(msg):
    info = {}
    info["received_at"] = midcatime.now()
    for pair in msg.split("|"):
        pair = pair.strip()
        if not pair:
            continue #ignore whitespace; e.g. after final '|'
        try:
            key, value = pair.split(":")
        except:
            raise Exception("Improperly formatted feedback received: " + s)
            return
        key = key.strip()
        value = value.strip()
        try:
            value = float(value)
        except:
            pass #not a number; fine.
        info[key] = value
    return info

class RosMidca:

    def __init__(self, midcaObj, incomingMsgHandlers = [], outgoingMsgHandlers = [],
    dynamicPublishers = False):
        self.midca = midcaObj
        self.incomingMsgHandlers = incomingMsgHandlers
        self.outgoingMsgHandlers = outgoingMsgHandlers
        self.dynamicPublishers = False

    def run_midca(self, cycleRate = 10):
        self.midca.init()
        cycleRate = rospy.Rate(cycleRate)
        while not rospy.is_shutdown():
            try:
                self.midca.next_phase(verbose = 2)
            except rospy.ROSInterruptException:
                break
            cycleRate.sleep()

    def ros_connect(self):
        rospy.init_node("MIDCA")
        for handler in self.incomingMsgHandlers:
            handler.subscriber = rospy.Subscriber(handler.topic, handler.msgType, handler.callback)
        for handler in self.outgoingMsgHandlers:
            handler.publisher = rospy.Publisher(handler.topic, handler.msgType, queue_size = 10)
        global rosMidca
        rosMidca = self


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
        print "trying to send message on topic", topic
        sent = False
        for handler in self.outgoingMsgHandlers:
            if handler.topic == topic:
                if handler.publisher == None:
                    rospy.logerr("publisher for topic " + str(topic) + " has not been " +
                    "initialized. Msg " + str(msg) + " Will not be sent.")
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
                            str(msg) + " is of type " + str(type(msg)) + " where " +
                            str(handler.msgType) + " was expected. Msg will not be sent.")
                            convertedMsg = None
                        else:
                            convertedMsg = msg
                    if convertedMsg:
                        rospy.loginfo("Midca sending msg: " + str(convertedMsg))
                        handler.publisher.publish(convertedMsg)
                        sent = True
        return sent


class IncomingMsgHandler(object):

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
        msgType = PointStamped
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
        loc = pointMsg.point))

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
        print "storing utterance:", utterance
        if not self.mem:
            rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
        self.mem.add(self.memKey, world_repr.UtteranceEvent(utterance.data.strip()))

class FeedbackHandler(IncomingMsgHandler):

    '''
    Handler that accepts feedback from ROS about commands issued. String is formatted as:
    argName: value | argName: value | ...
    Where argName is a string and value can be a string or number - if float(value)
    succeeds it is interpreted as a number; otherwise a string.
    Common args are ('cmd_id': name of command issued by MIDCA), ('midcatime': midcatime at which MIDCA
    sent the command as reported by MIDCA), ('code': code representing success,
    different failure types, in-progress, etc.)
    '''

    def __init__(self, topic, midcaObject, memKey = None):
        callback = lambda strMsg: self.store_feedback(strMsg)
        msgType = String
        super(FeedbackHandler, self).__init__(topic, msgType, callback, midcaObject)
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ROS_FEEDBACK

    def store_feedback(self, msg):
        if not self.mem:
            rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
        s = msg.data
        try:
            self.mem.add(self.memKey, s)
        except:
            print "Error reading feedback: ", s, " - format should be key: value | key : \
             value..."

class OutgoingMsgHandler(object):

    def __init__(self, topic, msgType):
        self.topic = topic
        self.msgType = msgType
        self.publisher = None
        self.subscriber = None
