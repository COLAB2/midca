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


def load_domain(domainfile, problemfile):
    (dom, prob) = pddl.parseDomainAndProblem(domainfile, problemfile)
    print()
    print("DOMAIN PROBLEM")
    # print("objects")
    ###############types#########################
    print('types: ')

    for arg in dom.types.args:
        type(arg.arg_name)

    type("constant")

    for t in types:
        print(t.__repr__())

    ###Predicates##########
    print('predicates: ')
    for a in dom.predicates:
        argnames = parseTypedArgList_names(a.args)
        predicate(a.name, argnames, a.args)


    ######### OPERATORS ####################

    print('actions:')

    for a in dom.actions:

        actions_args = parseTypedArgList(a.parameters)

        prepredicates = []
        postpredicates = []
        preobjnames = []
        postobjnames = []
        preobjtypes = []
        postobjtypes = []
        prepos = []
        postpos = []
        for pre in a.get_pre(True):
            # a.args is typedArgList
            prepos.append(True)
            pre_args_name = parseTypedArgList_names(pre.args)

            pre_args_type = parseTypedArgList_types(pre.args, actions_args)

            prepredicates.append(worldsim.Predicate(pre.name, pre_args_name, pre_args_type))

            preobjnames.append(pre_args_name)
            preobjtypes.append(pre_args_type)
            # for p in range(0 , len(pre_args_type)):
            #     print(pre_args_name[p])
            #     print(pre_args_type[p].__str__())

        # for pre in a.get_pre(False):
        #     # a.args is typedArgList
        #     print(pre)
        #     # pre_args_name = parseTypedArgList_names(pre.args)
        #     # pre_args_type = parseTypedArgList_types(pre.args, actions_args)
        #     # prepredicates.append(worldsim.Predicate(pre.name, pre_args_name))
        #     # preobjnames.append(pre_args_name)
        #     # preobjtypes.append(pre_args_type)

        for eff in a.get_eff(True):
            postpos.append(True)
            # a.args is typedArgList
            eff_args_names = parseTypedArgList_names(eff.args)
            eff_args_types = parseTypedArgList_types(eff.args, actions_args)

            postpredicates.append(worldsim.Predicate(eff.name, eff_args_names, eff_args_types))

            postobjnames.append(eff_args_names)
            postobjtypes.append(eff_args_types)

        # for eff in a.get_eff(False):
        #     # a.args is typedArgList
        #     print(eff)
        #     # eff_args_names = parseTypedArgList_names(eff.args)
        #     # eff_args_types = parseTypedArgList_types(eff.args, actions_args)
        #     # postpredicates.append(worldsim.Predicate(eff.name, eff_args_names))
        #     # postobjnames.append(eff_args_names)
        #     # postobjtypes.append(eff_args_types)

        operators.update({a.name :worldsim.Operator(a.name, list(actions_args.keys()), prepredicates, preobjnames, preobjtypes, prepos,
                                             postpredicates, postobjnames, postobjtypes, postpos)})

    print("Objects:")
    objects = parseObjects(prob.objects)

    world = worldsim.World(list(operators.values()), list(predicates.values()), atoms, types, list(objects.values()), cltree, obtree)

    return world

    # probinitialState = getInitialState(prob.initialstate)


def getInitialState(probinitialState):

    for a in probinitialState:
        if type(a) is FExpression:
            print("feexpression")
            print(a.op)
            for sub in a.subexps:
                if type(sub) is FHead:
                    print(sub.name)
                    print(parseTypedArgList_names(sub.args))
                else:
                    print(sub.val)

        else:
            print("formula")
            print(a.op)
            for sub in a.subformulas:
                print(sub)
                print(sub.name)
                print(parseTypedArgList_names(sub.args))
        # goal = prob.goal
        # print(goal)






def instance(name, typename):
    if typename not in types:
        raise Exception("object type DNE.")
    objects[name] = types[typename].instantiate(name)

def type(name, parentnames=["obj"]):
    temp = [name]
    if not parentnames == ["obj"]:
        temp.append(parentnames)
    if isinstance(parentnames, str):
        parentnames = [parentnames]
    parents = []
    for parent in parentnames:
        if parent not in types:
            raise Exception("parent type DNE.")
        parents.append(types[parent])
    types[name] = worldsim.Type(name, parents)
    otree = worldsim.ObjectTree(obtree['rootnode'],
                                obtree['allnodes'],
                                obtree['checked'],
                                temp)
    obtree['rootnode'] = otree.rootnode
    obtree['allnodes'] = otree.allnodes
    obtree['checked'] = otree.checked

def parseObjects(objects):
    worldsimObjects = {}
    for arg in objects.args:
        worldsimObjects.update({arg.arg_name: worldsim.Obj(arg.arg_name, types[arg.arg_type])})

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
        if arg.arg_type in types:
            parsed.update({arg.arg_name: types[arg.arg_type]})
        else:
            parsed.update({arg.arg_name: types["resource"]})
    return parsed

def parseTypedArgList_names(argList):
    parsed = []
    for arg in argList.args:
        parsed.append(str(arg.arg_name))
    return parsed

def predicate(name, argnames, argList):
    argtypes = []
    for arg in argList.args:
        if arg.arg_type not in types:
            argtypes.append(types["resource"])
        else:
             argtypes.append(types[arg.arg_type])

    predicates[name] = worldsim.Predicate(name, argnames, argtypes)
    for t in predicates[name].argtypes:
        print(t.__str__())

def parseTypedArgList_types_predicate(argList):
    ptypes = []
    for arg in argList.args:
        if arg.arg_type in types.keys():
            ptypes.append(types[arg.arg_type])
        else:

            ptypes.append(types["resource"])

    return ptypes

def parseTypedArgList_types(argList, action_types):
    ptypes = []
    for arg in argList.args:
        if arg.arg_name in action_types.keys():
            ptypes.append(action_types[arg.arg_name])
        else:

            ptypes.append(types["resource"])

    return ptypes

if __name__ == "__main__":
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"

    ### Domain Specific Variables for JSHOP planner
    ff_DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/sminecraft.pddl"
    ff_STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/s_wood.pddl"

    load_domain(ff_DOMAIN_FILE, ff_STATE_FILE)
