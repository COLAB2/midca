
#source: https://github.com/hfoffani/pddl-lib
#I ran this using Python 3.6 and ubuntu
#this script might not work on windows.

#install pip3
#python3.6 -m pip install

#install pddlpy
#pip3 install pddlpy

#I had to delete antlr4-python3-runtime and install it with this command:
#pip3 install antlr4-python3-runtime

import midca.worldsim.pddl_worldsim as worldsim
import pddlpy
from  pddlpy import DomainProblem
import inspect, os


objects = []
predicates = []
atoms = []
operators = []
cltree = {"rootnode": "", "allnodes": [], "checked": []}
obtree = {"rootnode": "", "allnodes": [], "checked": []}

def load_domain(domain_file, problem_file):
    domprob = pddlpy.DomainProblem(domain_file, problem_file)
    pddl_operators = list(domprob.operators())
    print(pddl_operators)
    for op in pddl_operators:
        print("start...")
        #'move'
        name = op
        prepredicates = []
        postpredicates = []
        preobjnames = []
        postobjnames = []
        # {'?from': 'm4_1', '?to': 'm4_1'}
        variable_list = list(domprob.ground_operator(op))[0].variable_list
        objnames = variable_list.keys()

        #ex:{('connect', 'm4_1', 'm4_1'), ('player-at', 'm4_1')}
        # {('tool-in-hand', '?tool'), ('player-at', '?target'), ('thing-at-map', 'tree', '?target')}
        precondition_pos = list(domprob.ground_operator(op))[0].precondition_pos
        for pre in precondition_pos:
            prename = pre[0]
            args =[]
            for p in pre[1:]:
                args.append(p)

            prepredicates.append(worldsim.Predicate(prename, args))
            preobjnames.append(args)


        precondition_neg = list(domprob.ground_operator(op))[0].precondition_neg

        for pre in precondition_neg:
            prename = pre[0]
            args =[]
            for p in pre[1:]:
                args.append(p)

            prepredicates.append(worldsim.Predicate(prename, args))
            preobjnames.append(args)

        print('pre is done')
        effect_pos = list(domprob.ground_operator(op))[0].effect_pos

        for post in effect_pos:
            postname = post[0]
            args =[]
            for p in post[1:]:
                args.append(p)

            postpredicates.append(worldsim.Predicate(postname, args))
            postobjnames.append(args)

        effect_neg = list(domprob.ground_operator(op))[0].effect_neg

        for post in effect_neg:
            postname = post[0]
            args =[]
            for p in post[1:]:
                args.append(p)

            postpredicates.append(worldsim.Predicate(postname, args))
            postobjnames.append(args)

        print('post is done')

    # operators = dic {'stack':worldsim.operator}

        operators.append(worldsim.Operator(op, objnames, prepredicates, preobjnames, postpredicates, postobjnames))
        print("end")

    predicates = prepredicates + postpredicates

    world = worldsim.World(operators, predicates, atoms)

    return world

def readpddlfiles(domainfile, problemfile):
    domprob = DomainProblem(domainfile, problemfile)
    print()
    print("DOMAIN PROBLEM")
    # print("objects")
    # print("\t", domprob.worldobjects())
    print("operators")
    print("\t", list(domprob.operators()))
    # print("init", )
    # print("\t", domprob.initialstate())
    print("goal", )
    print("\t", domprob.goals())

    print()
    ops_to_test = {1: "op2", 2: "move", 3: "move", 4: "move", 5: "A1"}
    op = ops_to_test[2]
    print("ground for operator", op, "applicable if (adjacent loc1 loc2)")
    for op in domprob.operators():
        name = op
        preconditions = list(domprob.ground_operator(op))[0].precondition_pos

        print(name)

        print(preconditions)

        print(list(domprob.ground_operator(op))[0].variable_list.keys())
    # for o in domprob.ground_operator(op):
    #
    #     print()
    #     print("\tvars", o.variable_list)
    #     print("\tpre+", o.precondition_pos)
    #     print("\tpre-", o.precondition_neg)
    #     print("\teff+", o.effect_pos)
    #     print("\teff-", o.effect_neg)


if __name__ == "__main__":
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"


    ### Domain Specific Variables for JSHOP planner
    ff_DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/sminecraft.pddl"
    ff_STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/s_wood.pddl"

    load_domain(ff_DOMAIN_FILE,ff_STATE_FILE)
