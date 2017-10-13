from midca import base
from midca.worldsim import domainread, stateread, worldsim, blockstate, scene
from midca.modules import simulator, perceive, note, guide, evaluate, intend, planning, act
import os, inspect

def asqiiDisplay(world):
	blocks = blockstate.get_block_list(world)
	print str(scene.Scene(blocks))

def UserGoalsMidca(domainFile, stateFile, goalsFile = None, extinguish = False):
	world = domainread.load_domain(domainFile)
	stateread.apply_state_file(world, stateFile)
	myMidca = base.PhaseManager(world, display = asqiiDisplay)
	for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)

	myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
	myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
	myMidca.append_module("Perceive", perceive.PerfectObserver())
	myMidca.append_module("Interpret", note.ADistanceAnomalyNoter(5, 0.1))
	myMidca.append_module("Interpret", guide.UserGoalInput())
	myMidca.append_module("Eval", evaluate.SimpleEval())
	myMidca.append_module("Intend", intend.SimpleIntend())
	myMidca.append_module("Plan", planning.PyHopPlanner(extinguish))
	myMidca.append_module("Act", act.SimpleAct())
	return myMidca

def guiMidca(domainFile, stateFile, goalsFile = None):
	world = domainread.load_domain(domainFile)
	stateread.apply_state_file(world, stateFile)
	myMidca = base.PhaseManager(world, display = asqiiDisplay)
	for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)

	myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
	myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
	myMidca.append_module("Perceive", perceive.PerfectObserver())
	myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
	myMidca.append_module("Eval", evaluate.SimpleEval())
	myMidca.append_module("Intend", intend.SimpleIntend())
	myMidca.append_module("Plan", planning.PyHopPlanner())
	myMidca.append_module("Act", act.SimpleAct())
	return myMidca

if __name__ == "__main__":
	thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
	MIDCA_ROOT = thisDir + "/../"
	myMidca = UserGoalsMidca(domainFile = MIDCA_ROOT + "worldsim/domains/arsonist.sim", stateFile = MIDCA_ROOT + "worldsim/states/defstate.sim")
	myMidca.init()
	myMidca.run()