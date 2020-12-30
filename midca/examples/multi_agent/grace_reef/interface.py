from multiprocessing import Process
import json, time
import zmq

class RosConnection:
    pass

class ZmqConnection:

    def __init__(self):
        pass

    def __init__(self, config):
        self.config = config
        self.establish_connections()

    def subscriber_connection(self, ip_address):
        """
        :param ip_address: address to connect
        :return: subscriber object
        """
        subscriber = ""
        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)
        subscriber.setsockopt(zmq.SUBSCRIBE, "")
        subscriber.setsockopt(zmq.RCVTIMEO, 3)
        #subscriber.setsockopt(zmq.CONFLATE, 1)
        subscriber.connect(ip_address)

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
        self.config["publish_interface"] = self.subscriber_connection(self.config["publish_interface"])

        for agent in self.config["agents"]:

            # interface to agent publishing connections
            for topic in self.config["agents"][agent]["address"]:
                self.config["agents"][agent]["address"][topic] = self.publisher_connection(self.config["agents"][agent]["address"][topic])

            # interface to simulator publishing connections
            for topic in self.config["agents"][agent]["publish_simulator"]:
                self.config["agents"][agent]["publish_simulator"][topic] = self.publisher_connection(self.config["agents"][agent]["publish_simulator"][topic])

            # simulator to interface subscribing connections
            for topic in self.config["agents"][agent]["subscribe_simulator"]:
                self.config["agents"][agent]["subscribe_simulator"][topic] = self.subscriber_connection(self.config["agents"][agent]["subscribe_simulator"][topic])

        print self.config

    def publish_messages(self, msg, publisher_object):
        """
        :param publisher_object: zmq publisher object
        :return:
        """
        publisher_object.send_multipart(message)

    def recieve_messages(self, subscriber_object):
        """
        :param subscriber_object: zmq subscriber object
        :return:
        """
        try:
            message = subscriber_object.recv()
            return message
        except Exception as e:
            print e
            time.sleep(0.1)
            return None


    def get_all_messages(self, subscriber_object):
        msg = "True"
        all_messages = []
        while (msg):
            msg = self.recieve_messages(subscriber_object)
            if msg:
                all_messages.append(msg)

        return all_messages

    def start(self):

        while (True):
            # get messages from agents
            agents_messages =  self.get_all_messages(self.config["publish_interface"])

            # get all messages from the simulator
            for agent in self.config["agents"]:
                for topic in self.config["agents"][agent]["subscribe_simulator"]:
                    simulator_messages = self.get_all_messages(self.config["agents"][agent]["subscribe_simulator"][topic])
                    print (simulator_messages)



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
            agent_process = Process(target=run_script, args=(agent['script'],))
            agent_process.start()
            #agent_process.join()

    def run_simulator(self, config):
        """
        :param filter: filter functionality
               config:
        :return:
        """
        connection_type = ""
        if config["connection_type"] == "zmq":
            message_service = ZmqConnection(config)
            message_service.start()


if __name__ == '__main__':
    interface = Interface()
    config = interface.read_config()
    interface.run_simulator(config)
    #run_agent_scripts(config)
    #run_simulator(config, filter)


    """"
    message_service = ZmqConnection(config)
    sub = message_service.config["agents"]["grace"]["subscribe_simulator"]["agent"]
    while (True):
        try:
            message = sub.recv()
            print (message)
        except:
            pass
    """
