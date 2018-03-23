
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

from pythonpddl import pddl
import inspect, os


objects = []
predicates = []
atoms = []
operators = []
cltree = {"rootnode": "", "allnodes": [], "checked": []}
obtree = {"rootnode": "", "allnodes": [], "checked": []}



def readpddlfiles(domainfile, problemfile):
    (dom, prob) = pddl.parseDomainAndProblem(domainfile, problemfile)
    print()
    print("DOMAIN PROBLEM")
    # print("objects")

    print('types: ')
    for arg in dom.types.args: #TypedArg
        print(str(arg.arg_name) + " " + str(arg.arg_type))

    print('predicates: ')
    for a in dom.predicates:
        print(a.name)
        #a.args is typedArgList
        for arg in a.args.args:
            print(str(arg.arg_name) + " " + str(arg.arg_type))

    # for a in dom.actions:
    #     print(a.name)
    #     for x in a.get_pre(True):
    #         print(x.asPDDL())

    # for a in dom.actions:
    #     print(a.name, "c",  list(map(lambda x: x.asPDDL(), a.get_pre(True))))
    #
    #     print(a.name, "e", list(map(lambda x: x.asPDDL(), a.get_eff(True))))




    # ops_to_test = {1: "op2", 2: "move", 3: "move", 4: "move", 5: "A1"}
    # op = ops_to_test[2]
    # print("ground for operator", op, "applicable if (adjacent loc1 loc2)")
    # for op in domprob.operators():
    #     name = op
    #     preconditions = list(domprob.ground_operator(op))[0].precondition_pos
    #
    #     print(name)
    #
    #     print(preconditions)


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
    ff_DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
    ff_STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.75.pddl"

    readpddlfiles(ff_DOMAIN_FILE,ff_STATE_FILE)
