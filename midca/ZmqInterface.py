from midca import midcatime
import math
import numpy as np
import time
import zmq
import threading
import traceback
import os

class ZmqMidca:

    def __init__(self, midcaObj, incomingMsgHandlers = [], outgoingMsgHandlers = [],
    dynamicPublishers = False):
        self.midca = midcaObj
        self.incomingMsgHandlers = incomingMsgHandlers
        self.outgoingMsgHandlers = outgoingMsgHandlers
        self.dynamicPublishers = False

    def run_incoming_handlers(self):
        for handler in self.incomingMsgHandlers:
            threading.Thread(target=handler.run).start()

    def run_outgoing_handlers(self):
        for handler in self.outgoingMsgHandlers:
            threading.Thread(target=handler.run).start()

    def run_midca(self, cycleRate = 0.5, maxPhaseLength = 30):
        #self.midca.init()
        while True:
            try:
                self.midca.next_phase(verbose = self.midca.verbose)
                if self.midca.mem.metaEnabled:
                        metaval = self.midca.one_cycle(verbose = self.midca.verbose, pause=0.01, meta=True)
                time.sleep(cycleRate)
            except KeyboardInterrupt:
                print ("KeyboardInterrupt")
                os._exit(0)

            except Exception as e:
                print (e)
                traceback.print_exc()
                os._exit(0)


    def subscriber_connection(self, ip_address, connection="connect", connection_type = "sub"):
        """
        :param ip_address: address to connect
        :return: subscriber object
        """
        subscriber = ""
        context = zmq.Context()
        if connection_type == "sub":
            subscriber = context.socket(zmq.SUB)
            subscriber.setsockopt(zmq.SUBSCRIBE, "")
            subscriber.setsockopt(zmq.RCVTIMEO, -1)
        elif connection_type == "pull":
            subscriber = context.socket(zmq.SUB)
            subscriber.setsockopt(zmq.SUBSCRIBE, "")
            subscriber.setsockopt(zmq.RCVTIMEO, -1)
            subscriber.setsockopt(zmq.CONFLATE, 1)

        #subscriber.setsockopt(zmq.CONFLATE, 1)
        if connection == "connect":
            subscriber.connect(ip_address)
        else:
            subscriber.bind(ip_address)

        return subscriber

    def publisher_connection(self, ip_address, connection="connect", connection_type = "pub"):
        """
        :param ip_address: address to connect
        :return: publisher object
        """
        publisher = ""
        context = zmq.Context()
        publisher = context.socket(zmq.PUB)
        publisher.connect(ip_address)
        return publisher

    def connect(self):
        for handler in self.incomingMsgHandlers:
            handler.subscriber = self.subscriber_connection(handler.ip_address, handler.connection, handler.connection_type)
        for handler in self.outgoingMsgHandlers:
            handler.publisher = self.publisher_connection(handler.ip_address, handler.connection, handler.connection_type)
        self.run_incoming_handlers()
        self.run_outgoing_handlers()
        global zmqMidca
        zmqMidca = self

    """
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
    """

class IncomingMsgHandler(object):

    def __init__(self, ip_address, connection, connection_type, midcaObject = None):
        if midcaObject:
            self.mem = midcaObject.mem
        else:
            self.mem = None
        self.ip_address = ip_address
        self.connection = connection
        self.connection_type = connection_type
        self.subscriber = None

    def recieve_messages(self):
        """
        :param subscriber_object: zmq subscriber object
        :return:
        """
        try:
            message = self.subscriber.recv()
            return message
        except Exception as e:
            pass
            #print e
            time.sleep(0.1)

class OutgoingMsgHandler(object):

    def __init__(self, ip_address, connection, connection_type, midcaObject = None):
        if midcaObject:
            self.mem = midcaObject.mem
        else:
            self.mem = None
        self.ip_address = ip_address
        self.connection = connection
        self.connection_type = connection_type
        self.publisher = None
        self.subscriber = None

    def send_messages(self, msg):
        """
        :param subscriber_object: zmq subscriber object
        :return:
        """
        try:
            self.publisher.send_string(msg)
        except Exception as e:
            pass
            #print e
            time.sleep(0.1)
