import copy
import shlex, subprocess

class Goal:

    def __init__(self, *args, **kwargs):
        if 'id' in kwargs:
            self.id = kwargs['id']
        else:
            self.id = hash(self)
        self.args = args
        self.kwargs = kwargs

    def __getitem__(self, val):
        if val in self.kwargs:
            return self.kwargs[val]
        else:
            try:
                return self.args[val]
            except TypeError:
                #not an index
                raise KeyError(str(val) + " is not a valid key or index.")

    def __contains__(self, item):
        try:
            val = self[item]
            return True
        except KeyError:
            return False

    def __setitem__(self, key, item):
        if isinstance(key, int):
            if key < len(self.args):
                self.args[key] = item
            else:
                raise IndexError(str(key) + " is out of range for args of " + str(self))
        else:
            self.kwargs[key] = item

    def get_args(self):
        '''
        Return the arguments for this goal.
        Example:
        Goal(A_,B_, on)
        would return [A_,B_] as the args
        '''
        return self.args
    
    def get_pred(self):
        '''
        Return the predicate for this goal.
        Example:
        Goal(A_,B_, on)
        would return 'on' as the args
        '''
        return self.kwargs['predicate']

    def __str__(self):
        s = "Goal(" + "".join([str(arg) + ", " for arg in self.args]) + "".join([str(key) + ": " + str(value) + ", " for key, value in self.kwargs.items()])
        if self.args or self.kwargs:
            return s[:-2] + ")"
        else:
            return s + ")"

class GoalNode:

    id = 1

    def __init__(self, goal):
        self.goal = goal
        self.parents = set()
        self.children = set()
        self.plan = None
        self.id = GoalNode.id
        GoalNode.id += 1

    def addChild(self, node):
        self.children.add(node)
        node.parents.add(self)

    def setPlan(self, plan):
        self.plan = plan

    def dotStr(self):
        """ Nice string format for labeling the graph in the pdf drawing """
        return str(self.goal)

class GoalGraph:

    '''
    A graph that maintains a partial ordering of goals. Note that, at present, cycle checking is not complete, so partial orderings can be created that would never allow a goal to be accomplished.
    The single constructor argument gives a function that takes two goals as input and should return a +/- value indicating precedence. If goal1 should be achieved before goal2, goalCompareFunction(goal1, goal2) < 0.
    '''

    def __init__(self, goalCompareFunction = None):
        self.roots = set()
        self.cmp = goalCompareFunction
        if not self.cmp:
            self.cmp = lambda goal1, goal2: 0
        self.numGoals = 0
        self.plans = set()

    #note not symmetrical - finds goals that are specifications of current goal, but not generalizations.
    def consistentGoal(self, first, second):
        for i in range(len(first.args)):
            if first.args[i] != "?" and (len(second.args) <= i or first.args[i] != second.args[i]):
                return False
        for key, val in first.kwargs.items():
            if key not in second.kwargs or second.kwargs[key] != val:
                return False
        return True

    def sameGoal(self, goal1, goal2):
        return type(goal1) == type(goal2) and goal1.args == goal2.args and goal1.kwargs == goal2.kwargs

    def add(self, goal):
        self.insert(goal)

    #inserts a goal into the graph using the graph's comparator
    def insert(self, goal):
        if goal in self:
            return False
        newNode = GoalNode(goal)
        self.numGoals += 1
        if not self.roots:
            self.roots.add(newNode)
        for node in self._getAllNodes():
            cmpVal = self.cmp(newNode.goal, node.goal)
            if cmpVal < 0:
                newNode.addChild(node)
            elif cmpVal > 0:
                node.addChild(newNode)
        self.roots = {node for node in self.roots if node not in newNode.children}
        if not newNode.parents:
            self.roots.add(newNode)
        if not self.roots:
            raise ValueError("Adding a goal that creates a cycle in the graph. Now no goals can be achieved.")
        return True

    def _removeNode(self, delNode):
        self.numGoals -= 1
        allNodes = self._getAllNodes()
        if delNode in self.roots:
            self.roots.remove(delNode)
            for node in allNodes:
                if delNode in node.parents:
                    node.parents.remove(delNode)
                    if not node.parents and node != delNode:
                        self.roots.add(node)
                if delNode in node.children:
                    node.children.remove(delNode)

    def remove(self, goal):
        delNode = self._getGoalNode(goal)
        if not delNode:
            return
        self._removeNode(delNode)

    def addPlan(self, plan):
        self.plans.add(plan)

    #removes all goals associated with given plan. Not super efficient right now, but the expectation is that the number of goals will not be huge.
    def removePlanGoals(self, plan):
        for goal in plan.goals:
            self.remove(goal)

    #will raise KeyError if plan is not in plan set.
    def removePlan(self, plan):
        self.plans.remove(plan)

    def planCurrent(self, plan, requireAllGoals = True):
        numGoalsMissed = 0
        for goal in plan.goals:
            if not self._getGoalNode(goal):
                if requireAllGoals:
                    return False
                else:
                    numGoalsMissed += 1
        if numGoalsMissed == len(plan.goals):
            return False
        return True

    def removeOldPlans(self, requireAllGoals = True):
        self.plans = {plan for plan in self.plans if self.planCurrent(plan, requireAllGoals)}

    def numMatchingGoals(self, plan, goals):
        num = 0
        for goal in goals:
            found = False
            for planGoal in plan.goals:
                if self.sameGoal(goal, planGoal):
                    found = True
                    break
            if found:
                num += 1
        return num

    #returns all plan whose goalset contains any of given goals. Will return them in order of how many given goals they achieve, ties broken by minimizing extra goals. Note that this ordering may break if a plan has more than a thousand goals.
    def allMatchingPlans(self, goals):
        matches = []
        for plan in self.plans:
            if self.numMatchingGoals(plan, goals) > 0:
                matches.append(plan)
        matches.sort(key = lambda plan: -self.numMatchingGoals(plan, goals) + len(plan.goals) * 0.001)
        return matches

    #returns a plan whose goalset contains all given goals. If more than one plan does, returns one of those with minimum extraneous goals. Ties are broken arbitrarily. If there is no candidate, returns None.
    def getMatchingPlan(self, goals):
        bestChoice = None
        for plan in self.plans:
            goalMissing = False
            for goal in goals:
                found = False
                for planGoal in plan.goals:
                    #print "goal is " + str([goal])
                    if self.consistentGoal(goal, planGoal):
                        found = True
                        break
                if not found:
                    goalMissing = True
                    break
            if not goalMissing:
                if not bestChoice:
                    bestChoice = plan
                elif len(bestChoice.goals) > len(plan.goals):
                    bestChoice = plan
        return bestChoice

    #returns the plan, if any is available, that achieves the most goals in the given goalset. If more than one does, tries to achieve the fewest extraneous goals. Ties are broken arbitrarily. Returns None if no plan is found that achieves any of the given goals.
    #note that this method is a generalization of getMatchingPlan() (i.e. will return a best matching plan if there is any), but is less efficient.
    def getBestPlan(self, goals):
        bestChoice = None
        bestNumAchieved = 0
        for plan in self.plans:
            numAchieved = 0
            for goal in goals:
                found = False
                for planGoal in plan.goals:
                    if self.consistentGoal(goal, planGoal):
                        found = True
                        break
                if found:
                    numAchieved += 1
            #check if the current plan achieves more goals than the best so far
            if numAchieved > bestNumAchieved:
                bestChoice = plan
                bestNumAchieved = numAchieved
            #break ties by minimizing total goals
            elif numAchieved == bestNumAchieved and len(bestChoice.goals) > len(plan.goals):
                bestChoice = plan
        return bestChoice

    def _getAllNodes(self):
        visited = set()
        nodes = list(self.roots)
        while nodes:
            next = nodes.pop(0)
            if next in visited:
                continue
            else:
                visited.add(next)
            for child in next.children:
                nodes.append(child)
        return visited

    def getAllGoals(self):
        visited = set()
        goals = []
        nodes = list(self.roots)
        while nodes:
            next = nodes.pop(0)
            if next in visited:
                continue
            else:
                visited.add(next)
            goals.append(next.goal)
            for child in next.children:
                nodes.append(child)
        return goals

    #returns the first node such that self.consistentGoal(goal, node.goal) returns True.
    def _getGoalNode(self, goal):
        visited = set()
        nodes = list(self.roots)
        while nodes:
            next = nodes.pop(0)
            if next in visited:
                continue
            else:
                visited.add(next)
            if self.consistentGoal(goal, next.goal):
                return next
            for child in next.children:
                nodes.append(child)
        return None #not in graph

    def getGoalAncestors(self, goal):
        node = self._getGoalNode(goal)
        if node:
            ancestors = set()
            nodes = [node]
            while nodes:
                next = nodes.pop(0)
                if next in ancestors:
                    continue
                for parent in next.parents:
                    ancestors.add(parent)
                    nodes.append(parent)
            return ancestors
        else:
            raise ValueError("Goal not in graph")

    def __contains__(self, goal):
        for _goal in self.getAllGoals():
            if self.sameGoal(goal, _goal):
                return True
        return False

    def __str__(self):
        return "Goals: " + str([str(goal) + " " for goal in self.getAllGoals()])

    def getUnrestrictedGoals(self):
	'''
	sort the nodes according to the node.id and return node.goal accordingly
	'''
	nodes_list = []
	# get all the nodes of root into a list for sorting
        for node in self.roots:
	     nodes_list.append(node)
	# sort according to the node id
	nodes_list.sort(key = lambda x: x.id)
	return [node.goal for node in nodes_list]

    def writeToPDF(self, pdf_filename="goalgraph.pdf"):
        """ Requires the 'dot' command be installed on the current system. To
            install on unix simply type 'sudo apt-get install
            graphviz'

            The filename must end in .pdf . A temporary .dot
            file will be made and then removed to create the
            pdf file. The path for the pdf file will be the
            same for the .dot file.

            Since this function traverses the graph, it is
            important that there is not a cycle. If there is,
            then one of the relationships in the cycle may not
            described in the graph

            Note that this could create a potential security
            vulnerability if the filename of the pdf passed in
            is prepended with malicious code.

            To-do list:
            - put everything in a directory

        """

        assert(pdf_filename.endswith(".pdf"))

        # get the filename for dot by removing '.pdf'
        dotfilename = copy.deepcopy(pdf_filename[0:-4]) + ".dot"
        dotfilestr = "digraph\n{\n"

        for node in self._getAllNodes():
            print("  Goal" + str(node.id) + " [label=\""+node.dotStr()+" \"]")
            dotfilestr += "  Goal" + str(node.id) + " [label=\""+node.dotStr()+" \"]\n"

        dotfilestr += "\n"

        for node in self._getAllNodes():
            for node_child in node.children:
                dotfilestr += "  Goal" + str(node.id) + " -> Goal" + str(node_child.id) + " \n"

        dotfilestr += "\n}\n"
        f = open(dotfilename, 'w')
        f. write(dotfilestr)
        f.close()
        #print "Wrote dot file to " + dotfilename
        genPDFCommand = "dot -Tpdf "+ dotfilename + " -o " + pdf_filename
        #dot_output = subprocess.check_output(shlex.split(genPDFCommand))
        #print "dot_output = " + str(dot_output)
        #subprocess.call(shlex.split("del "+dotfilename))
        print "Drawing of current goal graph written to " + pdf_filename

