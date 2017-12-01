from __future__ import print_function
from midca import base
from midca.worldsim import domainread, stateread, worldsim
from midca.modules import simulator, perceive, note, guide, evaluate, intend, planning, act


def UserGoalsMidca(domainFile, stateFile, display=print, goalsFile = None, argsPyHopPlanner=[]):
    world = domainread.load_domain(domainFile)
    stateread.apply_state_file(world, stateFile)
        #creates a PhaseManager object, which wraps a MIDCA object
    myMidca = base.PhaseManager(world, display = display, verbose=4)
        #add phases by name
    for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
        myMidca.append_phase(phase)

        #add the modules which instantiate basic blocksworld operation
    myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
    myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(display))
    myMidca.append_module("Perceive", perceive.PerfectObserver())
    myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
    #myMidca.append_module("Interpret", guide.UserGoalInput())
    myMidca.append_module("Eval", evaluate.SimpleEval())
    myMidca.append_module("Intend", intend.SimpleIntend())
    myMidca.append_module("Plan", planning.PyHopPlanner(*argsPyHopPlanner))
    myMidca.append_module("Act", act.SimpleAct())
    return myMidca

# ~-~- OLD ~-~-
# def guiMidca(domainFile, stateFile, goalsFile = None):
#     world = domainread.load_domain(domainFile)
#     stateread.apply_state_file(world, stateFile)
#     myMidca = base.PhaseManager(world, display = asqiiDisplay)
#     for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
#         myMidca.append_phase(phase)
# 
#     myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
#     myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
#     myMidca.append_module("Perceive", perceive.PerfectObserver())
#     myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
#     myMidca.append_module("Eval", evaluate.SimpleEval())
#     myMidca.append_module("Intend", intend.SimpleIntend())
#     myMidca.append_module("Plan", planning.PyHopPlanner())
#     myMidca.append_module("Act", act.SimpleAct())
#     return myMidca

