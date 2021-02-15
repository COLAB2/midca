from difflib import SequenceMatcher

class Filter:

    def __init__(self, config, connection):
        self.config = config
        # An object of class ZmqConnection or RosConnection from interface.py
        self.connection = connection

    def getAgentAddress(self, msg):
        """
        param msg: message
        functionality: find the agent name from the message and
                       using config variable get the publisher of the agent
        reutrn: agents publisher
        """
        agent_name = None
        features  = msg.split(",")
        for each in features:
            if "NAME" in each:
                agent_name = each.split(":")[-1].lower()

        return self.config["agents"][agent_name]["address"]["agent"]

    def filterSimulatorMsgs(self, messages):
        """
        :param messages: messages in string format
        :return: None
        :functionality: Filter messages from simulator and send it to appropriate agent
        """
        if messages:
            if len(messages) >= 2:
                match_percentage = SequenceMatcher(None, messages[-1], messages[-2]).ratio()*100
                if match_percentage >=90:
                    messages = messages[-1:]

            for msg in messages:
                agent_address = self.getAgentAddress(msg)
                self.connection.publish_messages(msg, agent_address)


    def recieveAgentMsgs(self, messages):
        """
        :param messages: messages in string format
        :return: None
        :functionality: Filter messages from agent and send it to appropriate simulator
        """
        pass
