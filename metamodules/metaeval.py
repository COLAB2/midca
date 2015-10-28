from MIDCA import base

class MRSimpleEval(base.BaseModule):

    def run(self, cycle, verbose):
        # check to see if there are any goals
        metagoals = self.mem.get(self.mem.META_GOALS)
        if metagoals and len(metagoals) > 0:
            for goal in metagoals:
                pass

        else:
            print("No goals - eval doing nothing")

    def goal_achieved(self, goal):
        pass