from midca import base
from midca.worldsim import domainread, stateread, worldsim
from midca.domains.blocksworld import blockstate, scene
from midca.modules.perceive import PerfectObserver
from midca.modules.plan import PyHopPlanner, PyHopPlannerBroken
from midca.modules.intend import SimpleIntend
from midca.modules.act import SimpleAct
from midca.modules.interpret import ADistanceAnomalyNoter, UserGoalInput
from midca.modules.evaluate import SimpleEval, Scorer
from midca.modules import simulator
from midca.metamodules import monitor, control, interpret, metaintend,  plan

def asqiiDisplay(world):
    blocks = blockstate.get_block_list(world)
    print(str(scene.Scene(blocks)))

def UserGoalsMidca(domainFile, stateFile, goalsFile = None, extinguish = False):
    world = domainread.load_domain(domainFile)
    stateread.apply_state_file(world, stateFile)
    myMidca = base.PhaseManager(world, verbose=1, display = asqiiDisplay, metaEnabled=True)

    # add cognitive layer phases
    for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
        myMidca.append_phase(phase)

    # add cognitive layer modules
    myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
    myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
    myMidca.append_module("Perceive", PerfectObserver.PerfectObserver())
    myMidca.append_module("Interpret", ADistanceAnomalyNoter.ADistanceAnomalyNoter())
    myMidca.append_module("Interpret", UserGoalInput.UserGoalInput())
    myMidca.append_module("Eval", SimpleEval.SimpleEval())
    myMidca.append_module("Intend", SimpleIntend.SimpleIntend())
    myMidca.append_module("Plan", PyHopPlannerBroken.PyHopPlannerBroken(extinguish))
    myMidca.append_module("Act", SimpleAct.SimpleAct())

    # add meta layer phases
    #for phase in ["Monitor", "Interpret", "Eval", "Intend", "Plan", "Control"]:
    for phase in ["Monitor", "Interpret", "Intend", "Plan", "Control"]:
        myMidca.append_meta_phase(phase)

    # add meta layer modules
    myMidca.append_meta_module("Monitor", monitor.MRSimpleMonitor())
    myMidca.append_meta_module("Interpret", interpret.MRSimpleDetect())
    myMidca.append_meta_module("Interpret", interpret.MRSimpleGoalGen())
    myMidca.append_meta_module("Intend", metaintend.MRSimpleIntend())
    myMidca.append_meta_module("Plan", plan.MRSimplePlanner())
    myMidca.append_meta_module("Control", control.MRSimpleControl())

    return myMidca

def guiMidca(domainFile, stateFile, goalsFile = None):
    world = domainread.load_domain(domainFile)
    stateread.apply_state_file(world, stateFile)
    myMidca = base.PhaseManager(world, display = asqiiDisplay)
    for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
        myMidca.append_phase(phase)

    myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
    myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
    myMidca.append_module("Perceive", PerfectObserver.PerfectObserver())
    myMidca.append_module("Interpret", ADistanceAnomalyNoter.ADistanceAnomalyNoter())
    myMidca.append_module("Eval", SimpleEval.SimpleEval())
    myMidca.append_module("Intend", SimpleIntend.SimpleIntend())
    myMidca.append_module("Plan", PyHopPlanner.PyHopPlanner())
    myMidca.append_module("Act", SimpleAct.SimpleAct())
    return myMidca

