# source: https://github.com/hfoffani/pddl-lib
# I ran this using Python 3.6 and ubuntu
# this script might not work on windows.

# install pip3
# python3.6 -m pip install

# install pddlpy
# pip3 install pddlpy

# I had to delete antlr4-python3-runtime and install it with this command:
# pip3 install antlr4-python3-runtime

import midca.worldsim.worldsim as worldsim

from pythonpddl import pddl
from pythonpddl.pddl import FExpression, FHead, ConstantNumber
import inspect, os

types = {"obj": worldsim.Type("obj", [])}
objects = {}
predicates = {}
atoms = []
operators = {}
cltree = {"rootnode": "", "allnodes": [], "checked": []}
obtree = {"rootnode": "", "allnodes": [], "checked": []}


def readpddlfiles(domainfile, problemfile):
    (dom, prob) = pddl.parseDomainAndProblem(domainfile, problemfile)
    print()
    print("DOMAIN PROBLEM")
    # print("objects")
    ###############types#########################
    print('types: ')
    for arg in dom.types.args:  # TypedArg
        print(str(arg.arg_name) + " " + str(arg.arg_type))
        types.update({arg.arg_name: worldsim.Type(arg.arg_name)})

    ###Predicates##########
    print('predicates: ')
    for a in dom.predicates:
        print(a.name)
        argnames = parseTypedArgList_names(a.args)
        argtypes = parseTypedArgList_types(a.args)

        predicates.update({a.name: worldsim.Predicate(a.name, argnames, argtypes)})

    ######### OPERATORS ####################

    print('actions:')

    for a in dom.actions:
        print(a.name)
        actions_args = parseTypedArgList(a.parameters)

        prepredicates = []
        postpredicates = []
        preobjnames = []
        postobjnames = []
        preobjtypes = []
        postobjtypes = []

        for pre in a.get_pre(True):
            # a.args is typedArgList
            pre_args_name = parseTypedArgList_names(pre.args)

            pre_args_type = []
            constantType = worldsim.Type("constant")
            for p in pre_args_name:
                if p in actions_args.keys():
                    pre_args_type.append(actions_args[p])
                else:
                    pre_args_type.append(constantType)

            prepredicates.append(worldsim.Predicate(pre.name, pre_args_name, pre_args_type))

            preobjnames.append(pre_args_name)
            preobjtypes.append(pre_args_type)

        # for pre in a.get_pre(False):
        #     # a.args is typedArgList
        #     print(pre)
        #     # pre_args_name = parseTypedArgList_names(pre.args)
        #     # pre_args_type = parseTypedArgList_types(pre.args)
        #     # prepredicates.append(worldsim.Predicate(pre.name, pre_args_name))
        #     # preobjnames.append(pre_args_name)
        #     # preobjtypes.append(pre_args_type)

        for eff in a.get_eff(True):
            # a.args is typedArgList
            eff_args_names = parseTypedArgList_names(eff.args)
            eff_args_types = []
            constantType = worldsim.Type("constant")
            for p in eff_args_names:
                if p in actions_args.keys():
                    eff_args_types.append(actions_args[p])
                else:
                    eff_args_types.append(constantType)


            postpredicates.append(worldsim.Predicate(eff.name, eff_args_names, eff_args_types))

            postobjnames.append(eff_args_names)
            postobjtypes.append(eff_args_types)

        # for eff in a.get_eff(False):
        #     # a.args is typedArgList
        #     print(eff)
        #     # eff_args_names = parseTypedArgList_names(eff.args)
        #     # eff_args_types = parseTypedArgList_types(eff.args)
        #     # postpredicates.append(worldsim.Predicate(eff.name, eff_args_names))
        #     # postobjnames.append(eff_args_names)
        #     # postobjtypes.append(eff_args_types)

        operators[a.name] = worldsim.Operator(a.name, list(actions_args.values()), prepredicates, preobjnames, preobjtypes, [],
                                             postpredicates, postobjnames, postobjtypes, [])

        print("Objects:")
        objects = parseObjects(prob.objects)
        probinitialState = prob.initialstate
        for a in probinitialState:
            if type(a) is FExpression:
                print(a.op)
                for sub in a.subexps:
                    if type(sub) is FHead:
                        print(sub.name)
                        print(parseTypedArgList_names(sub.args))
                    else:
                        print(sub.val)

            else:
                print(a.op)
                for sub in a.subformulas:
                    print(sub.name)
                    print(parseTypedArgList_names(sub.args))
        # goal = prob.goal
        # print(goal)

def parseObjects(objects):
    worldsimObjects = {}
    for arg in objects.args:
        worldsimObjects.update({arg.arg_name: worldsim.Obj(arg.arg_name, worldsim.Type(arg.arg_type))})

    return worldsimObjects

def pasrsPredicate(dompredicates):
    parsed = []
    for a in dompredicates:
        # a.args is typedArgList
        predicate_args = parseTypedArgList_names(a.args)
        parsed.append(a.name + " " + predicate_args)
    return parsed

def parseTypedArgList(argList):
    parsed = {}
    for arg in argList.args:
        parsed.update({arg.arg_name: worldsim.Type(arg.arg_type)})
    return parsed

def parseTypedArgList_names(argList):
    parsed = []
    for arg in argList.args:
        parsed.append(str(arg.arg_name))
    return parsed

def parseTypedArgList_types(argList):
    parsed = []
    for arg in argList.args:
        parsed.append(worldsim.Type(arg.arg_type))

    return parsed

if __name__ == "__main__":
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"

    ### Domain Specific Variables for JSHOP planner
    ff_DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
    ff_STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.75.pddl"

    readpddlfiles(ff_DOMAIN_FILE, ff_STATE_FILE)
