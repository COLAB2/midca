import json
import pickle

class Node():
    '''
    Abstract State or an action
    '''

    def __init__(self, predicate, *args):
        self.predicate = predicate
        self.args = args



class Explanation_pattern():

    '''
    An explanation pattern
    '''

    def __init__(self):
        '''
        Each explanation pattern contains explains_node, pre_xp nodes and xp_asserted nodes
        '''
        self.prexp_nodes = {}
        self.explains_node = {}
        self.xp_asserted_nodes = {}
        self.probability = 0



    def create_node(self,state_or_action):
        '''

        :param world: Contains state
        :return: created node
        '''


        # first replace he
        preprocessing = state_or_action.replace("(", " ")
        preprocessing = preprocessing.replace(",", "")
        preprocessing = preprocessing.replace(")", "")
        preprocessing = preprocessing.split(" ")

        #print preprocessing
        predicate =preprocessing[0]
        args = preprocessing[1:]


        return Node(predicate,args)

    def create_explanation_pattern(self, prexp_nodes,
                                         explains_node,
                                         xp_asserted_nodes,
                                         likelihood):
        '''

        :param prexp_nodes: Observed states/actions in the world
        :param explains_node: observed anomalous state/action in the world
        :param xp_asserted_nodes: hypothetical states/actions in the world
        :param likelihood: The likelihood that it is selected
        :return: the explanation pattern
        '''

        # get all the prexp_nodes
        # check if all the prexp_nodes are possible in the state
        for each in prexp_nodes:
            # create node
            node = self.create_node(each)
            self.prexp_nodes[node] = 0

        # get the explains_node
        # check if it is possible in the state
        exp_node = self.create_node(explains_node)
        self.explains_node = exp_node

        # get the xp-asserted nodes
        for each in xp_asserted_nodes:
            node = self.create_node(each)
            self.xp_asserted_nodes[node] = 0

        # get the likelihood
        self.probability = likelihood



class Explanations:

    '''
     A mini version of meta aqua to work with midca
    '''

    def __init__(self):
        self.explanations = []


    def read_explanations(self, path="/home/sampath/Documents/git/midca/midca/domains/moos_domain/explanations1.json"):
        '''
        :param path: Json file from the path
        :return:
        '''

        with open(path) as case_base:
            casebase = json.load(case_base)
            for xp in casebase["Explanations"]:
                explanation_pattern = Explanation_pattern()
                explanation_pattern.create_explanation_pattern(xp["PREXP_NODES"],
                                           xp["EXPLAINS_NODE"],
                                           xp["XPASSERTED_NODES"],
                                            0)
                self.explanations.append(explanation_pattern)


    def determine_anomaly(self, world):
        '''

        :param world: states
        :return: explanations
        '''

        explanations = []
        for xp in self.explanations:
            for atom in world.atoms:
                if atom.predicate.name == xp.explains_node.predicate:
                    xp.explains_node.args = atom.args
                    explanations.append(xp)
                    break

        return explanations

    def get_candidate_explanations(self, world, actions, explanations):
        '''

        :param world: Get states for the world
        :return: the candidate explanations
        '''

        candidate_explanations = []
        for xp in explanations:
                count = 0
                for node in xp.prexp_nodes:
                    if actions:
                        for action in actions:
                            if node.predicate == action.op:
                                node.args = action.args
                                count = count + 1
                                break
                    for atom in world.atoms:
                        if node.predicate == atom.predicate.name:
                            node.args = atom.args
                            count = count + 1
                            break

                if len(xp.prexp_nodes) == count:
                    candidate_explanations.append(xp)

        return candidate_explanations



    def retrieval(self, world, actions=None):
        '''

        :param world: States of the world and actions
        :return: subset of explanations
        '''

        # get explanations that match anomaly
        explanations = self.determine_anomaly(world)

        # get candidates from explanation
        candidate_explanations = self.get_candidate_explanations( world, actions, explanations)

        return candidate_explanations


def main():
    xps = Explanations()
    xps.read_explanations()
    filehandler = open("/home/sampath/Documents/git/midca/midca/examples/states.obj", 'r')
    actionhandler = open("/home/sampath/Documents/git/midca/midca/examples/actions.obj", 'r')
    world = pickle.load(filehandler)
    actions = pickle.load(actionhandler)
    candidates = xps.retrieval(world,actions)


if __name__== "__main__":
  main()

