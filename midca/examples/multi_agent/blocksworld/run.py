from multiprocessing import Process
import json, time
import zmq
from filter import Filter
import threading
import sys

class RosConnection:
    pass

class ZmqConnection:

    def __init__(self):
        pass

    def __init__(self, config):
        self.config = config
        self.establish_connections()

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
            subscriber.setsockopt(zmq.RCVTIMEO, 1)
        elif connection_type == "pull":
            subscriber = context.socket(zmq.PULL)
            subscriber = context.socket(zmq.PULL)
            subscriber.setsockopt(zmq.RCVTIMEO, -1)
            subscriber.setsockopt(zmq.CONFLATE, 1)

        #subscriber.setsockopt(zmq.CONFLATE, 1)
        if connection == "connect":
            subscriber.connect(ip_address)
        else:
            subscriber.bind(ip_address)

        return subscriber

    def publisher_connection(self, ip_address):
        """
        :param ip_address: address to connect
        :return: publisher object
        """
        publisher = ""
        context = zmq.Context()
        publisher = context.socket(zmq.PUB)
        publisher.connect(ip_address)
        return publisher

    def establish_connections(self):
        """
        :functionality : add subscriber and publisher objects to self.config file
        :return: modify self.config
        """
        # agent to interface subscribtion
        self.config["publish_interface"] = self.subscriber_connection(self.config["publish_interface"], "bind")

        for agent in self.config["agents"]:

            # interface to agent publishing connections
            for topic in self.config["agents"][agent]["address"]:
                self.config["agents"][agent]["address"][topic] = self.publisher_connection(self.config["agents"][agent]["address"][topic])

            # interface to simulator publishing connections
            for topic in self.config["agents"][agent]["publish_simulator"]:
                self.config["agents"][agent]["publish_simulator"][topic] = self.publisher_connection(self.config["agents"][agent]["publish_simulator"][topic])

            # simulator to interface subscribing connections
            for topic in self.config["agents"][agent]["subscribe_simulator"]:
                self.config["agents"][agent]["subscribe_simulator"][topic] = self.subscriber_connection(self.config["agents"][agent]["subscribe_simulator"][topic], connection_type = "pull")

        #print (self.config)

    def publish_messages(self, msg, publisher_object):
        """
        :param publisher_object: zmq publisher object
        :return:
        """
        if type(msg) == str:
            publisher_object.send_string(msg)
        else:
            publisher_object.send_pyobj(msg)

    def recieve_messages(self, subscriber_object):
        """
        :param subscriber_object: zmq subscriber object
        :return:
        """
        try:
            message = subscriber_object.recv_pyobj()
            return message
        except Exception as e:
            pass
            #print e
            time.sleep(0.1)

        try:
            message = subscriber_object.recv()
            return message
        except Exception as e:
            pass
            #print e
            time.sleep(0.1)





    def get_all_messages(self, subscriber_object):
        msg = "True"
        all_messages = []
        while (msg):
            msg = self.recieve_messages(subscriber_object)
            if msg:
                if all_messages and str(msg) == str(all_messages[-1]):
                    break
                else:
                    all_messages.append(msg)


        return all_messages

    def createAgentThreadReceiveMsgs(self, subscriber, filter):
        while (True):
            agents_messages =  self.get_all_messages(subscriber)
            if agents_messages:
                filter.filterAgentMsgs(agents_messages)

    def createSimulatorThreadReceiveMsgs(self, subscriber, agent, filter):
        while (True):
            simulator_messages = self.get_all_messages(subscriber)
            filter.filterSimulatorMsgs(simulator_messages, agent)

    def start(self, filter):
        """
        param: filter: an object of python class from filter.py to
                       filter messages and send it to appropriate agent
        """
        # get messages from agents
        subscriber =  self.config["publish_interface"]
        threading.Thread(target=self.createAgentThreadReceiveMsgs, args=(subscriber, filter, )).start()

        # get all messages from the simulator
        for agent in self.config["agents"]:
            for topic in self.config["agents"][agent]["subscribe_simulator"]:
                subscriber =  self.config["agents"][agent]["subscribe_simulator"][topic]
                threading.Thread(target=self.createSimulatorThreadReceiveMsgs, args=(subscriber,agent,filter,)).start()




class Interface:

    def read_config(self, filename = "config.json"):
        """
        input: filename
        :return: json object
        """
        config = None
        with open(filename) as f:
            config = json.load(f)
        return config

    def run_script(self, script, args=None):
        """
        :param script: the path of the script
        :param args: commandline arguments to run the script
        :return: none
        """
        print (script)
        f = open(script)
        exec (f.read())


    def run_agent_scripts(self, config):
        """
        :param config: configuration parameters
        :functionality: run agent scripts
        """
        agents = config['agents']
        for agent in agents:
            agent_process = Process(target=self.run_script, args=(agents[agent]['script'],))
            agent_process.start()
            #agent_process.join()

    def run_simulator(self, config, filter):
        """
        :param filter: an object of python class from filter.py to
                       filter messages and send it to appropriate agent
               config:
        :return:
        """
        connection_type = ""
        if config["connection_type"] == "zmq":
            message_service = ZmqConnection(config)
            filter = Filter(config, message_service)
            message_service.start(filter)


if __name__ == '__main__':
    interface = Interface()
    config = interface.read_config()
    interface.run_simulator(config, filter)
    flag = "--withagents"
    if len(sys.argv) > 1:
        flag =  sys.argv[1]

    if flag=="--withoutagents":
        pass
    else:
        interface.run_agent_scripts(config)

    # Todo: have to efficiently implement the thread handling
    # to continously run the threads
    while (True):
        pass
    #run_agent_scripts(config)
    #run_simulator(config, filter)
