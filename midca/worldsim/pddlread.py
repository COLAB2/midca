
#source: https://github.com/hfoffani/pddl-lib
#I ran this using Python 3.6 and ubuntu
#this script might not work on windows.

#install pip3
#python3.6 -m pip install

#install pddlpy
#pip3 install pddlpy

#I had to delete antlr4-python3-runtime and install it with this command:
#pip3 install antlr4-python3-runtime

import midca.worldsim.worldsim as worldsim
import pddlpy
import inspect, os

def readpddlfiles(domain_file, problem_file):
    domprob = pddlpy.DomainProblem(domain_file, problem_file)
    initstate = domprob.initialstate()

    operators = list(domprob.operators())
    print(operators)

    print(list(domprob.ground_operator('move'))[0].precondition_pos)


if __name__ == "__main__":
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"


    ### Domain Specific Variables for JSHOP planner
    ff_DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/sminecraft.pddl"
    ff_STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/s_wood.pddl"

    readpddlfiles(ff_DOMAIN_FILE,ff_STATE_FILE)
