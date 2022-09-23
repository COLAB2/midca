from midca import base,goals
import copy
import os
import socket
import logging

class RemoteObserver(base.BaseModule, object):
    """
    Copies the agent state to MIDCA memory, allowing for custom fog-of-war.

    Unlike the non-remote version, the world given to this module should be
    an instance of MIDCAClient, and is an intermediary. The observer does not
    have direct access to the world.
    """

    def init(self, world, mem):
        """Give module access to critical MIDCA information."""
        self.mem = mem
        self.client = world

    def observe(self):
        """Tell the agent to look at its surroundings."""
        return self.client.copy()
        self.logger.info("Observed world")

    def run(self, cycle, verbose=2):
        world = self.observe()
        if not world:
            raise Exception("World observation failed.")
        self.mem.add(self.mem.STATES, world)
        curr_goal=[]
        curr_goal_sets=[]
        #adding the goals to goal graph when it finds requested predicate
        while(self.mem.GOAL_READ != 1):
            for atom in world.atoms:
                if atom.predicate.name == "requested":
                    curr_goal.append(str(atom))
            curr_goals=[]
            self.mem.GOAL_READ = 1
            for x in curr_goal:
                x=x[26:]
                x=x.replace("not-","!")
                x=x.replace("native-at","native-at(")
                x=x.replace("invasive-at","invasive-at(")
                x=x.replace(", ","")
                curr_goals.append(x)
            for x in curr_goals:
                print(x)
                if x.startswith('!'):
                    negate = True
                    x = x[1:]
                else:
                    negate = False
                predicateName = x[:x.index("(")]
                #print(type(goals))
                args = [arg.strip() for arg in x[x.index("(") + 1:-1].split(",")]
                #print(type(args),type(predicateName))
                if negate:
                    curr_goal_sets.append(goals.Goal(*args, predicate=predicateName, negate=True))
                else:
                    curr_goal_sets.append(goals.Goal(*args, predicate=predicateName))

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)

        if len(self.mem.get(self.mem.GOAL_GRAPH).getAllGoals()) == 0:
            # get the list of goals
            goal_set = curr_goal_sets
            # insert that goal
            for g in goal_set:
                self.mem.get(self.mem.GOAL_GRAPH).insert(g)
		#self.mem.set(self.mem.Maintenance_goal, self.curr_goal_sets) 
            # update trace
            if trace:
                trace.add_data("NEXT GOAL(s)", goal_set)
                trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        else:
            if trace:
                trace.add_data("NEXT GOAL", 'goals not empty; no goal chosen')
                trace.add_data("GOAL GRAPH", copy.deepcopy(self.mem.GOAL_GRAPH))
        
        #print(str(self.mem.get(self.mem.GOAL_GRAPH)))
        

        # Memory Usage Optimization (optional, feel free to comment
        # drop old memory states if not being used
        # this should help with high memory costs
        states = self.mem.get(self.mem.STATES)
        #print("these are states", world)
        if len(states) > 400:
            # print "trimmed off 200 old stale states"
            states = states[200:]
            self.mem.set(self.mem.STATES, states)
        # End Memory Usage Optimization

        if verbose >= 1:
            print("World observed.")

        trace = self.mem.trace
        if trace:
            trace.add_module(cycle, self.__class__.__name__)
            trace.add_data("WORLD", copy.deepcopy(world))

