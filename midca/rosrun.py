

from midca.modules._robot_world import world_repr
from midca import midcatime
import math
import numpy as np


try:
    import rospy
    from geometry_msgs.msg import Point, PointStamped
    from std_msgs.msg import String
    import baxter_interface
    import baxter_external_devices
    from baxter_interface import CHECK_VERSION
    from scipy.spatial import distance
except:
    pass

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
    
    def run_midca(self, cycleRate = 13, maxPhaseLength = 30):
        self.midca.init()
        cycleRate = rospy.Rate(cycleRate)
        while not rospy.is_shutdown():
            try:
                self.midca.next_phase(verbose = self.midca.verbose)
                if self.midca.mem.metaEnabled:
                        metaval = self.midca.one_cycle(verbose = self.midca.verbose, pause=0.01, meta=True)
            except rospy.ROSInterruptException:
                break
            cycleRate.sleep()
    
    def ros_connect(self):
        rospy.init_node("MIDCA")
        for handler in self.incomingMsgHandlers:
            handler.subscriber = rospy.Subscriber(handler.topic, handler.msgType, handler.callback)
        for handler in self.outgoingMsgHandlers:
            handler.publisher = rospy.Publisher(handler.topic, handler.msgType, queue_size = 13)
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

class ObjectsLocationHandler(IncomingMsgHandler):

    '''
    class that receives Point messages, where each message indicates that the given object
    has been identified at that location. Args include the topic to listen to, object id,
    and midca object to whose memory observations will be stored. Optionally the memory
    key to be stored to can also be specified.
    '''
    
    def __init__(self, topic, midcaObject, memKey = None):
        callback = lambda strMsg: self.store_locations(strMsg)
        msgType = String
        super(ObjectsLocationHandler, self).__init__(topic, msgType, callback,
        midcaObject)
        #self.objID = objID
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ROS_OBJS_DETECTED
        
    def store_locations(self, data):
        if not self.mem:
            rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
        
        strMsg = str(data.data).strip()
        color_locations = strMsg.split(";")
        color_location_dic = {}
        for msg in color_locations:
            
            color_location = msg.split(":");
            
            pointstr = color_location[1].split(",")
            p = Point(x = int(pointstr[0]), y = int(pointstr[1]), z = int(pointstr[2]))
            color_location_dic.update({color_location[0]: p})
            
                
            self.mem.add(self.memKey, world_repr.DetectionEvent(id = color_location[0], 
        loc = p))
        
        if color_location_dic and ('red block' in color_location_dic.keys()) and ('green block' in color_location_dic.keys()):
            pos_green = 'table'
            pos_red = 'table'
            clear_green = 'clear'
            clear_red = 'clear'
            if math.fabs(color_location_dic['red block'].x - color_location_dic['green block'].x) < 12:
                if color_location_dic['red block'].y > color_location_dic['green block'].y:
                    pos_green = "red block"
                    clear_red = 'not clear'
                else:
                    pos_red = "green block"
                    clear_green = 'not clear'     
                
                self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "red block", position = pos_red, isclear = clear_red))
                self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "green block", position = pos_green, isclear = clear_green)) 
        
        elif len(color_location) == 1:
            color_block = color_location_dic.keys()[0]
            pos = 'table'
            clear = 'clear'
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = color_block, position = pos, isclear = clear))
        
#         if 'red block' in color_location_dic.keys() and color_location_dic['red block']:
#             if 'green block' in color_location_dic.keys() and color_location_dic['green block']:
#                 pos_green = 'table'
#                 pos_red = 'table'
#                 clear_green = 'clear'
#                 clear_red = 'clear'
#                 if math.fabs(color_location_dic['red block'].x - color_location_dic['green block'].x) < 10:
#                     if color_location_dic['red block'].y > color_location_dic['green block'].y:
#                         pos_green = "red block"
#                         clear_red = 'not clear'
#                     else:
#                         pos_red = "green block"
#                         clear_green = 'not clear'     
#                 
#                 self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "red block", position = pos_red, isclear = clear_red))
#                 self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "green block", position = pos_green, isclear = clear_green))    



class threeObjectsLocationHandler(IncomingMsgHandler):

    '''
    class that receives Point messages, where each message indicates that the given object
    has been identified at that location. Args include the topic to listen to, object id,
    and midca object to whose memory observations will be stored. Optionally the memory
    key to be stored to can also be specified.
    '''
    
    def __init__(self, topic, midcaObject, memKey = None):
        
        callback = lambda strMsg: self.store_locations(strMsg)
        msgType = String
        super(threeObjectsLocationHandler, self).__init__(topic, msgType, callback,
        midcaObject)
        #self.objID = objID
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ROS_OBJS_DETECTED
        
    def store_locations(self, data):



        if not self.mem:
            rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
        left = baxter_interface.Gripper('left')
        strMsg = str(data.data).strip()
        color_locations = strMsg.split(";")
        color_location_dic = {}
        for msg in color_locations:
            
            color_location = msg.split(":");
            
            pointstr = color_location[1].split(",")
            p = Point(x = int(pointstr[0]), y = int(pointstr[1]), z = int(pointstr[2]))
            color_location_dic.update({color_location[0]: p})
            
                
            self.mem.add(self.memKey, world_repr.DetectionEvent(id = color_location[0], 
        loc = p))

        '''
        This is the three stacking condition , the idea is checking the distance between the coordinates of the three blocks
        '''
        
        if color_location_dic and ('red block' in color_location_dic.keys()) and ('green block' in color_location_dic.keys()) and ('blue block' in color_location_dic.keys()):
            
            pos_green = 'table'
            pos_red = 'table'
            pos_blue = 'table'
            clear_green = 'clear'
            clear_red = 'clear'
            clear_blue = 'clear'
            r = (color_location_dic['red block'].x , color_location_dic['red block'].y)
            g = (color_location_dic['green block'].x , color_location_dic['green block'].y)
            b = (color_location_dic['blue block'].x , color_location_dic['blue block'].y)
            distancerg = distance.euclidean(r,g)
            distancerb = distance.euclidean(r,b)
            distancegb = distance.euclidean(g,b)
            xdistancerg= math.fabs(color_location_dic['red block'].x - color_location_dic['green block'].x)
            xdistancerb= math.fabs(color_location_dic['red block'].x - color_location_dic['blue block'].x)
            xdistancegb= math.fabs(color_location_dic['blue block'].x - color_location_dic['green block'].x)
            ydistancerg= math.fabs(color_location_dic['red block'].y - color_location_dic['green block'].y)
            ydistancerb= math.fabs(color_location_dic['red block'].y - color_location_dic['blue block'].y)
            ydistancegb= math.fabs(color_location_dic['blue block'].y - color_location_dic['green block'].y)
            edistance = [distancerg,distancerb,distancegb]
            ydistance = [color_location_dic['red block'].y,color_location_dic['blue block'].y ,color_location_dic['green block'].y]
            xdistance = [color_location_dic['red block'].x,color_location_dic['blue block'].x ,color_location_dic['green block'].x]
            y1distance = [ydistancerg,ydistancerb,ydistancegb]
            
            '''
            print("xdistancerg" + str(xdistancerg))
            print("xdistancerb" + str(xdistancerb))
            print("xdistancegb" + str(xdistancegb))
            print("ydistancerg" + str(ydistancerg))
            print("ydistancerb" + str(ydistancerb))
            print("ydistancegb" + str(ydistancegb))
            
            
            print("red + green e" + str(distancerg) );
            print("red + blue e" + str(distancerb) );
            print("blue + green e" + str(distancegb) );
            '''
        
        



            

            
            if ((xdistancerg > 13) and (xdistancerb > 13) and (xdistancegb > 13)):
                pos_green = 'table'
                pos_red = 'table'
                pos_blue = 'table'
                clear_green = 'clear'
                clear_red = 'clear'
                clear_blue = 'clear'


                

            elif ((xdistancerg <= 13) and (xdistancerb <= 13) and (xdistancegb <= 13) and (ydistancerg <= 45) and (ydistancerb <= 45) and (ydistancegb <= 45) and (distancerg <= 45) and (distancerb <= 45) and (distancegb <= 45) and ((ydistancerg > 28.5) or (ydistancerb > 28.5) or (ydistancegb > 28.5)  ) ):
                add =0 
                for i in edistance:
                    add = add + i;
                result = 2*(max(edistance)) - add
                if(result < 2):
                    maximum = max(ydistance)
                    firstpos = ''
                    secondpos = ''
                    temp = ydistance
                    if (maximum == color_location_dic['red block'].y):
                        pos_red = 'table'
                        clear_red = 'not clear'
                        firstpos = 'red block'
                    elif(maximum == color_location_dic['green block'].y):
                        pos_green = 'table'
                        clear_green = 'not clear'
                        firstpos = 'green block'
                    else:
                        pos_blue = 'table'
                        clear_blue = 'not clear'
                        firstpos = 'blue block'
                    temp.remove(maximum)
                    maximum = max(temp)
                    if (maximum == color_location_dic['red block'].y):
                        pos_red = firstpos
                        clear_red = 'not clear'
                        secondpos = 'red block'
                    elif(maximum == color_location_dic['green block'].y):
                        pos_green = firstpos
                        clear_green = 'not clear'
                        secondpos = 'green block'
                    else:
                        pos_blue = firstpos
                        clear_blue = 'not clear'
                        secondpos = 'blue block'
                    temp.remove(maximum)
                    maximum = max(temp)
                    if (maximum == color_location_dic['red block'].y):
                        pos_red = secondpos
                        clear_red = 'clear'
                    elif(maximum == color_location_dic['green block'].y):
                        pos_green = secondpos
                        clear_green = 'clear'
                    else:
                        pos_blue = secondpos
                        clear_blue = 'clear'

            elif ( ((xdistancerg <= 13) and (xdistancerb <= 13) and (xdistancegb <= 13)) ^ ((ydistancerg <= 28.5) and (ydistancerb <= 28.5) and (ydistancegb <= 28.5) and (distancerg <= 28.5) and (distancerb <= 28.5) and (distancegb <= 28.5)) or ((xdistancerg <= 13) or (xdistancerb <= 13) or (xdistancegb <= 13)) and ((ydistancerg <= 28.5) or (ydistancerb <= 28.5) or (ydistancegb <= 28.5)) ):
                if(xdistancerg <= 13):
                    temps = ydistance
                    temps.remove(color_location_dic['blue block'].y)
                    maximum = max(temps)
                    if (maximum == color_location_dic['red block'].y):
                        pos_red = 'table'
                        clear_red = 'not clear'
                        pos_green = 'red block'
                        clear_green = 'clear'
                        
                    else:
                        pos_green = 'table'
                        clear_green = 'not clear'
                        pos_red = 'green block'
                        clear_red = 'clear'
                    
                elif(xdistancerb <= 13):
                    temps = ydistance
                    temps.remove(color_location_dic['green block'].y)
                    maximum = max(temps)
                    if (maximum == color_location_dic['red block'].y):
                        pos_red = 'table'
                        clear_red = 'not clear'
                        pos_blue = 'red block'
                        clear_blue = 'clear'
                        
                    else:
                        pos_blue = 'table'
                        clear_blue = 'not clear'
                        pos_red = 'blue block'
                        clear_red = 'clear'

                elif(xdistancegb <= 13):
                    temps = ydistance
                    temps.remove(color_location_dic['red block'].y)
                    maximum = max(temps)
                    if (maximum == color_location_dic['blue block'].y):
                        pos_blue = 'table'
                        clear_blue = 'not clear'
                        pos_green = 'blue block'
                        clear_green = 'clear'

                        
                    else:

                        pos_green = 'table'
                        clear_green = 'not clear'
                        pos_blue = 'green block'
                        clear_blue = 'clear'
            

            '''
            This is the condition where two blocks are on table and if the hand of the robot is closed , we need to open it if there is no block on hand
            '''        

            
            if (left._state.position < 4 ):
                if left.ready() == True:
                    left.open()
            
            
            

            
            
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "blue block", position = pos_blue, isclear = clear_blue))                
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "red block", position = pos_red, isclear = clear_red))
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "green block", position = pos_green, isclear = clear_green)) 
        
            '''
            This is the two stacking condition , the idea is checking the distance between the coordinates of two  blocks
            '''
        elif color_location_dic and ('red block' in color_location_dic.keys()) and ('green block' in color_location_dic.keys()):        
            pos_green = 'table'
            pos_red = 'table'
            clear_green = 'clear'
            clear_red = 'clear'
            r = (color_location_dic['red block'].x , color_location_dic['red block'].y)
            g = (color_location_dic['green block'].x , color_location_dic['green block'].y)
            xdistancerg= math.fabs(color_location_dic['red block'].x - color_location_dic['green block'].x)
            ydistancerg= math.fabs(color_location_dic['red block'].y - color_location_dic['green block'].y)

            
            '''
            print("xdistancerg" + str(xdistancerg))
            print("xdistancerb" + str(xdistancerb))
            print("xdistancegb" + str(xdistancegb))
            print("ydistancerg" + str(ydistancerg))
            print("ydistancerb" + str(ydistancerb))
            print("ydistancegb" + str(ydistancegb))
            
            
            print("red + green e" + str(distancerg) );
            print("red + blue e" + str(distancerb) );
            print("blue + green e" + str(distancegb) );
            '''

                

            if ((xdistancerg <= 13)  and (ydistancerg <=28.5)):
                    if (color_location_dic['red block'].y > color_location_dic['green block'].y):
                        pos_red = 'table'
                        pos_green = 'red block'
                        clear_green = 'clear'
                        clear_red = 'not clear'
                    else:
                        pos_green = 'table'
                        pos_red = 'green block'
                        clear_red = 'clear'
                        clear_green = 'not clear'

            if not(left._state.position > 80):
                    pos_blue = 'in-arm'
                    clear_blue = 'not clear'
                    self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "blue block", position = pos_blue, isclear = clear_blue))    
                

                        
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "red block", position = pos_red, isclear = clear_red))
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "green block", position = pos_green, isclear = clear_green)) 
        
        elif color_location_dic and ('red block' in color_location_dic.keys()) and ('blue block' in color_location_dic.keys()): 
            pos_blue = 'table'
            pos_red = 'table'
            clear_blue = 'clear'
            clear_red = 'clear'
            r = (color_location_dic['red block'].x , color_location_dic['red block'].y)
            b = (color_location_dic['blue block'].x , color_location_dic['blue block'].y)
            xdistancerb= math.fabs(color_location_dic['red block'].x - color_location_dic['blue block'].x)
            ydistancerb= math.fabs(color_location_dic['red block'].y - color_location_dic['blue block'].y)

            
            '''
            print("xdistancerg" + str(xdistancerg))
            print("xdistancerb" + str(xdistancerb))
            print("xdistancegb" + str(xdistancegb))
            print("ydistancerg" + str(ydistancerg))
            print("ydistancerb" + str(ydistancerb))
            print("ydistancegb" + str(ydistancegb))
            
            
            print("red + green e" + str(distancerg) );
            print("red + blue e" + str(distancerb) );
            print("blue + green e" + str(distancegb) );
            '''

                

            if ((xdistancerb <= 13)  and (ydistancerb <=28.5)):
                    if (color_location_dic['red block'].y > color_location_dic['blue block'].y):
                        pos_red = 'table'
                        pos_blue = 'red block'
                        clear_blue = 'clear'
                        clear_red = 'not clear'
                    else:
                        pos_blue = 'table'
                        pos_red = 'blue block'
                        clear_red = 'clear'
                        clear_blue = 'not clear'

            # TODO: Use status of 'open' or 'closed' instead of > 80
            # TODO: This is not a good solution to solve this problem
            if not(left._state.position > 80):
                    pos_green = 'in-arm'
                    clear_green = 'not clear'
                    # TODO: BAD ASSUMPTION: if there are only two blocks, cannot assume y ou are holding the green block
                    self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "green block", position = pos_green, isclear = clear_green))
                
            
                        
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "red block", position = pos_red, isclear = clear_red))
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "blue block", position = pos_blue, isclear = clear_blue))

        elif color_location_dic and ('green block' in color_location_dic.keys()) and ('blue block' in color_location_dic.keys()): 
            pos_blue = 'table'
            pos_green = 'table'
            clear_blue = 'clear'
            clear_green = 'clear'
            r = (color_location_dic['green block'].x , color_location_dic['green block'].y)
            b = (color_location_dic['blue block'].x , color_location_dic['blue block'].y)
            xdistancegb= math.fabs(color_location_dic['green block'].x - color_location_dic['blue block'].x)
            ydistancegb= math.fabs(color_location_dic['green block'].y - color_location_dic['blue block'].y)

            
            '''
            print("xdistancerg" + str(xdistancerg))
            print("xdistancerb" + str(xdistancerb))
            print("xdistancegb" + str(xdistancegb))
            print("ydistancerg" + str(ydistancerg))
            print("ydistancerb" + str(ydistancerb))
            print("ydistancegb" + str(ydistancegb))
            
            
            print("red + green e" + str(distancerg) );
            print("red + blue e" + str(distancerb) );
            print("blue + green e" + str(distancegb) );
            '''

                

            if ((xdistancegb <= 13)  and (ydistancegb <=28.5)):
                    if (color_location_dic['green block'].y > color_location_dic['blue block'].y):
                        pos_green = 'table'
                        pos_blue = 'green block'
                        clear_blue = 'clear'
                        clear_green = 'not clear'
                    else:
                        pos_blue = 'table'
                        pos_green = 'blue block'
                        clear_green = 'clear'
                        clear_blue = 'not clear'

            if not(left._state.position > 80):
                    pos_red = 'in-arm'
                    clear_red = 'not clear'
                    self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "red block", position = pos_red, isclear = clear_red))    
                    
                
            
                        
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "green block", position = pos_green, isclear = clear_green))
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "blue block", position = pos_blue, isclear = clear_blue))


               


        
        elif len(color_location) == 1:
            color_block = color_location_dic.keys()[0]
            pos = 'table'
            clear = 'clear'
            self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = color_block, position = pos, isclear = clear))
        
#         if 'red block' in color_location_dic.keys() and color_location_dic['red block']:
#             if 'green block' in color_location_dic.keys() and color_location_dic['green block']:
#                 pos_green = 'table'
#                 pos_red = 'table'
#                 clear_green = 'clear'
#                 clear_red = 'clear'
#                 if math.fabs(color_location_dic['red block'].x - color_location_dic['green block'].x) < 13:
#                     if color_location_dic['red block'].y > color_location_dic['green block'].y:
#                         pos_green = "red block"
#                         clear_red = 'not clear'
#                     else:
#                         pos_red = "green block"
#                         clear_green = 'not clear'     
#                 
#                 self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "red block", position = pos_red, isclear = clear_red))
#                 self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = "green block", position = pos_green, isclear = clear_green))    


class MultipleObjectsLocationHandler(IncomingMsgHandler):

    '''
    class that receives Point messages, where each message indicates that the given object
    has been identified at that location. Args include the topic to listen to, object id,
    and midca object to whose memory observations will be stored. Optionally the memory
    key to be stored to can also be specified.
    '''
    
    def __init__(self, topic, midcaObject, memKey = None, history = None):
        callback = lambda strMsg: self.store_locations(strMsg)
        msgType = String
	self.left = None
        super(MultipleObjectsLocationHandler, self).__init__(topic, msgType, callback,
        midcaObject)
        #self.objID = objID
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.ROS_OBJS_DETECTED
	if history:
	    self.history = history
	else:
	    self.history = self.mem.STATE_HISTORY
        
    def store_locations(self, data):
        if not self.mem:
            rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
	if not self.left:
		rospy.sleep(0.1)
		self.left = baxter_interface.Gripper('left')
        
        strMsg = str(data.data).strip()
        color_locations = strMsg.split(";")
	color_locations.pop() # remove last unwanted ;
        color_location_dic = {}
        for msg in color_locations:
            
            color_location = msg.split(":");
            pointstr = color_location[1].split(",")
            p = Point(x = float(pointstr[0]), y = float(pointstr[1]), z = float(pointstr[2]))
            color_location_dic.update({color_location[0]: p})
            
                
            self.mem.add(self.memKey, world_repr.DetectionEvent(id = color_location[0], 
        loc = p))

	# complexity of this code is O(n2) will probably reduce some time
	# take each block and compare with all the blocks with certain thresholds
	# initially assume the block is on table and clear
	if color_location_dic :
		for each_block in color_location_dic:
			pos = 'table'
			clear = 'clear'
			for cmp_block in color_location_dic:
				if not each_block == cmp_block:
					# check stack condition
					x_difference = abs(color_location_dic[each_block].x - color_location_dic[cmp_block].x)
					y_difference = abs(color_location_dic[each_block].y - color_location_dic[cmp_block].y)
					z_difference = abs(color_location_dic[each_block].z - color_location_dic[cmp_block].z)
					if ( x_difference < 0.03 and y_difference < 0.01 and z_difference > 0.02 and z_difference < 0.06):
						if (color_location_dic[each_block].z < color_location_dic[cmp_block].z):
							clear = 'not clear'
						else:
							pos = cmp_block;
			# add it to the state
			self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = each_block, position = pos, isclear = clear))
			#if you want to know the position of the block, uncomment this
			#print ( each_block + ":  " +  pos + clear)
	# check if there is something in the baxter's hand
	found  = 0
	if self.left._state.position < 70:
		print(self.left._state.position)
		if self.mem.get(self.history):
			for each_history in self.mem.get(self.history):
				objects  = each_history
				for each_object in objects:
					if not each_object in color_location_dic:
						self.mem.add(self.mem.ROS_OBJS_STATE, world_repr.pos_block(id = each_object, position = 'holding', isclear = 'not clear'))
						found = 1						
						break
				if found == 1:
					break


class CalibrationHandler(IncomingMsgHandler):

    '''
    class that receives Point messages, where each message indicates that the given object
    has been identified at that location. Args include the topic to listen to, object id,
    and midca object to whose memory observations will be stored. Optionally the memory
    key to be stored to can also be specified.
    '''
    
    def __init__(self, topic, midcaObject, memKey = None):
        callback = lambda msg: self.store_calibrate_matrix(msg)
        msgType = String
        super(CalibrationHandler, self).__init__(topic, msgType, callback,
        midcaObject)
        
        if memKey:
            self.memKey = memKey
        else:
            self.memKey = self.mem.CALIBRATION_DONE
        
    def store_calibrate_matrix(self, msg):
        if not self.mem:
            rospy.logerr("Trying to store data to a nonexistent MIDCA object.")
        self.mem.set(self.mem.CALIBRATION_MATRIX, msg)
        


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
