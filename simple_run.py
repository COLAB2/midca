import phasemanager
from worldsim import domainread, stateread, worldsim
from modules import simulator, observe

world = domainread.load_domain("/Users/swordofmorning/Documents/_programming/repos/MIDCA/worldsim/domains/arsonist.sim")
stateread.apply_state_file(world, "/Users/swordofmorning/Documents/_programming/repos/MIDCA/worldsim/states/defstate.sim")
myMidca = phasemanager.MIDCA(world)
for phase in ["Simulate", "Observe", "Interpret", "Eval", "Intend", "Plan", "Act"]:
	myMidca.append_phase(phase)

myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
myMidca.append_module("Observe", observe.PerfectOberver())

myMidca.init()
myMidca.run()
