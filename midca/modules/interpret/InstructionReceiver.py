from midca import goals, base
from midca import midcatime
from ._goalgen import tf_3_scen, tf_fire
from midca.domains.logistics import deliverstate
from midca.domains.blocksworld import blockstate
from midca.worldsim import stateread
import copy,csv
import random
from midca.modules.monitors import Monitor
from threading import Thread

class InstructionReceiver:

    def init(self, world, mem):
        self.mem = mem
        self.lastTime = midcatime.now()

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATE)
        i = len(world.utterances)
        while i > 0:
            if self.lastTime - world.utterances[i - 1].midcatime > 0:
                break
            i -= 1
        newUtterances = [utterance.utterance for utterance in world.utterances[i:]]
        #now add goals based on new utterances
        for utterance in newUtterances:
            if verbose >= 2:
                print("received utterance:", utterance)
            if utterance == "point to the quad" or utterance == "goodbye baxter":
                goal = goals.Goal(objective = "show-loc", subject = "self",
                directObject = "quad", indirectObject = "observer")
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")
            if utterance == "get the red block":
                goal = goals.Goal(objective = "holding", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'red block:arm')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "get the green block":
                goal = goals.Goal(objective = "holding", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'green block:arm')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

                        if utterance == "get the blue block":
                            goal = goals.Goal(objective = "holding", subject = "self",
                                              directObject = "blue block", indirectObject = "observer", pos = 'blue block:arm')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")
            if utterance == "put the green block on table":
                goal = goals.Goal(objective = "moving", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'green block:table')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")
                        if utterance == "put the blue block on table":
                            goal = goals.Goal(objective = "moving", subject = "self",
                                  directObject = "blue block", indirectObject = "observer", pos = 'blue block:table')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "put the red block on table":
                goal = goals.Goal(objective = "moving", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'red block:table')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack the green block on the red block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'green block:red block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack the blue block on the red block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'blue block:red block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

                if utterance == "stack the blue block on the green block":
                    goal = goals.Goal(objective = "stacking", subject = "self",
                                      directObject = "green block", indirectObject = "observer", pos = 'blue block:green block')
                    added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

                        if utterance == "stack the red block on the blue block":
                            goal = goals.Goal(objective = "stacking", subject = "self",
                                              directObject = "blue block", indirectObject = "observer", pos = 'red block:blue block')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

                        if utterance == "stack the green block on the blue block":
                            goal = goals.Goal(objective = "stacking", subject = "self",
                                              directObject = "blue block", indirectObject = "observer", pos = 'green block:blue block')
                            added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \ goal graph")


            if utterance == "stack the red block on the green block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'red block:green block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'red block:green block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")



#             else:
#                 print "message is unknown"

        self.lastTime = midcatime.now()


class InstructionReceiver_sr:

    def init(self, world, mem):
        self.mem = mem
        self.lastTime = midcatime.now()

    def run(self, cycle, verbose = 2):
        world = self.mem.get(self.mem.STATE)
        i = len(world.utterances)
        while i > 0:
            if self.lastTime - world.utterances[i - 1].time > 0:
                break
            i -= 1
        newUtterances = [utterance.utterance for utterance in world.utterances[i:]]
        #now add goals based on new utterances
        for utterance in newUtterances:
            if verbose >= 2:
                print("received utterance:", utterance)
            if utterance == "point to the quad" or utterance == "goodbye baxter":
                goal = goals.Goal(objective = "show-loc", subject = "self",
                directObject = "quad", indirectObject = "observer")
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")
            if utterance == "get the red block":
                goal = goals.Goal(objective = "holding", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'red block:arm')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "get the green block":
                goal = goals.Goal(objective = "holding", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'green block:arm')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "get the blue block":
                goal = goals.Goal(objective = "holding", subject = "self",
                directObject = "blue block", indirectObject = "observer", pos = 'blue block:arm')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")
            if utterance == "put the green block on table":
                goal = goals.Goal(objective = "moving", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'green block:table')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")
            if utterance == "put the blue block on table":
                goal = goals.Goal(objective = "moving", subject = "self",
                directObject = "blue block", indirectObject = "observer", pos = 'blue block:table')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "put the red block on table":
                goal = goals.Goal(objective = "moving", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'red block:table')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack the green block on the red block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'green block:red block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack the blue block on the red block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "red block", indirectObject = "observer", pos = 'blue block:red block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack the blue block on the green block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'blue block:green block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack the red block on the blue block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "blue block", indirectObject = "observer", pos = 'red block:blue block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack the green block on the blue block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "blue block", indirectObject = "observer", pos = 'green block:blue block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \ goal graph")


            if utterance == "stack the red block on the green block":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'red block:green block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")

            if utterance == "stack":
                goal = goals.Goal(objective = "stacking", subject = "self",
                directObject = "green block", indirectObject = "observer", pos = 'red block:green block')
                added = self.mem.get(self.mem.GOAL_GRAPH).insert(goal)
                if verbose >= 2:
                    if added:
                        print("adding goal:", str(goal))
                    else:
                        print("generated goal:", str(goal), "but it is already in the \
                        goal graph")



#                       else:
#                               print "message is unknown"

        self.lastTime = midcatime.now()
