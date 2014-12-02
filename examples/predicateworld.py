from MIDCA import base
from MIDCA.worldsim import domainread, stateread, worldsim, blockstate, scene
from MIDCA.modules import simulator, observe, note, guide, evaluate, intend, planning, act

def asqiiDisplay(world):
	blocks = blockstate.get_block_list(world)
	print str(scene.Scene(blocks))

def UserGoalsMidca(domainFile, stateFile, goalsFile = None):
	world = domainread.load_domain(domainFile)
	stateread.apply_state_file(world, stateFile)
	myMidca = base.PhaseManager(world, display = asqiiDisplay)
	for phase in ["Simulate", "Observe", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)

	myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
	myMidca.append_module("Simulate", simulator.ASCIIWorldViewer())
	myMidca.append_module("Simulate", simulator.WorldChanger())
	myMidca.append_module("Observe", observe.PerfectOberver())
	myMidca.append_module("Interpret", note.ADistanceAnomalyNoter())
	myMidca.append_module("Interpret", guide.UserGoalInput())
	myMidca.append_module("Eval", evaluate.SimpleEval())
	myMidca.append_module("Intend", intend.SimpleIntend())
	myMidca.append_module("Plan", planning.PyHopPlanner())
	myMidca.append_module("Act", act.SimpleAct())
	return myMidca


