from difflib import SequenceMatcher

class Filter:

    def __init__(self, config, connection):
        self.config = config
        # An object of class ZmqConnection or RosConnection from interface.py
        self.connection = connection

        # to prevent repititive messages
        self.simulator_msgs = {}
        self.agent_messages = {}

    def getSimulatorAddress(self):
        """
        param msg: message
        functionality: find the agent name from the message and
                       using config variable get the publisher of the agent
        reutrn: agents publisher
        """
        return self.config["agents"]["neo"]["publish_simulator"]["simulator"]

    def getAgentAddress(self, agent_name):
        """
        param msg: agent_name
        reutrn: agents publisher
        """
        return self.config["agents"][agent_name]["address"]["agent"]

    def filterSimulatorMsgs(self, messages, agent):
        """
        :param messages: messages in string format
        :return: None
        :functionality: Filter messages from simulator and send it to appropriate agent
        """
        if messages:
            if len(messages) >= 2:
                match_percentage = SequenceMatcher(None, str(messages[-1]), str(messages[-2])).ratio()*100
                if match_percentage ==100:
                    messages = messages[-1:]

            messages_str = [str(each) for each in messages]
            if agent in self.simulator_msgs:
                if self.simulator_msgs[agent] == messages_str:
                    return
                else:
                    self.simulator_msgs[agent] = messages_str
            else:
                self.simulator_msgs[agent] = messages_str

            for msg in messages:
                agent_address = self.getAgentAddress(agent)
                print (agent, msg)
                self.connection.publish_messages(msg, agent_address)

    def filterAgentMsgs(self, messages):
        """
        :param messages: messages in string format
        :return: None
        :functionality: Filter messages from simulator and send it to appropriate agent
        """
        if messages:
            if len(messages) >= 2:
                match_percentage = SequenceMatcher(None, messages[-1], messages[-2]).ratio()*100
                if match_percentage ==100:
                    messages = messages[-1:]

            for msg in messages:
                print (msg)
                if "simulator" in msg:
                    msg = msg.replace("simulator:", "")
                    print ("simulator", msg)
                    simulator_address = self.getSimulatorAddress()
                    self.connection.publish_messages(msg, simulator_address)
                else:
                    msg = msg.split(";")
                    agent_name, msg = msg[0], msg[1]
                    agent_address = self.getAgentAddress(agent_name)
                    print (agent_name, msg)
                    self.connection.publish_messages(msg, agent_address)



    def recieveAgentMsgs(self, messages):
        """
        :param messages: messages in string format
        :return: None
        :functionality: Filter messages from agent and send it to appropriate simulator
        """
        pass
