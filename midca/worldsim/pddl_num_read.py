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
        argnames = []
        argtypes = []
        for arg in a.args.args:
            print(str(arg.arg_name) + " " + str(arg.arg_type))
            argnames.append(arg.arg_name)
            argtypes.append(arg.arg_type)

        predicates.update({a.name: worldsim.Predicate(a.name, argnames, argtypes)})

    ######### OPERATORS ####################

    print('actions:')
    parsed = []

    for a in dom.actions:
        print(a.name)
        actions_args = parseTypedArgList_names(a.parameters)
        prepredicates = []
        postpredicates = []
        preobjnames = []
        postobjnames = []
        preobjtypes = []
        postobjtypes = []

        for pre in a.get_pre(True):
            # a.args is typedArgList
            pre_args_name = parseTypedArgList_names(pre.args)
            pre_args_type = parseTypedArgList_types(pre.args)
            prepredicates.append(worldsim.Predicate(pre.name, pre_args_name))
            preobjnames.append(pre_args_name)
            preobjtypes.append(pre_args_type)

        for pre in a.get_pre(False):
            # a.args is typedArgList
            pre_args_name = parseTypedArgList_names(pre.args)
            pre_args_type = parseTypedArgList_types(pre.args)
            prepredicates.append(worldsim.Predicate(pre.name, pre_args_name))
            preobjnames.append(pre_args_name)
            preobjtypes.append(pre_args_type)

        for eff in a.get_eff(True):
            # a.args is typedArgList
            eff_args_names = parseTypedArgList_names(eff.args)
            eff_args_types = parseTypedArgList_types(eff.args)
            postpredicates.append(worldsim.Predicate(eff.name, eff_args_names))
            postobjnames.append(eff_args_names)
            postobjtypes.append(eff_args_types)

        for eff in a.get_eff(False):
            # a.args is typedArgList
            eff_args_names = parseTypedArgList_names(eff.args)
            eff_args_types = parseTypedArgList_types(eff.args)
            postpredicates.append(worldsim.Predicate(eff.name, eff_args_names))
            postobjnames.append(eff_args_names)
            postobjtypes.append(eff_args_types)

        operators[a.name] = worldsim.Operator(a.name, actions_args, prepredicates, preobjnames, preobjtypes, [],
                                            postpredicates, postobjnames, postobjtypes, [])


def pasrsPredicate(dompredicates):
    parsed = []
    for a in dompredicates:
        # a.args is typedArgList
        predicate_args = parseTypedArgList_names(a.args)
        parsed.append(a.name + " " + predicate_args)
    return parsed


def parseTypedArgList_names(argList):
    parsed = []
    for arg in argList.args:
        parsed.append(str(arg.arg_name))
    return parsed

def parseTypedArgList_types(argList):
    parsed = []
    for arg in argList.args:
        parsed.append(str(arg.arg_type))
    return parsed

if __name__ == "__main__":
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"

    ### Domain Specific Variables for JSHOP planner
    ff_DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
    ff_STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.75.pddl"

    readpddlfiles(ff_DOMAIN_FILE, ff_STATE_FILE)
