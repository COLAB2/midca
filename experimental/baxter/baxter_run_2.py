#this one uses moveit to move Baxter' arm

from MIDCA import base
from MIDCA.experimental.baxter import moveit_test, baxter

def print_func(s):
	print s

def baxter_midca():
	world = "Baxter World"
	myMidca = base.PhaseManager(world, display = print_func)
	for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)
	myMidca.append_module("Act", moveit_test.BaxterWave([], [(0.4, -0.2, 0.2), (0.5, -0.2, 0.4), (0.4, -0.2, 0.2)]))
	return myMidca

def test():
	myMidca = baxter_midca()
	myMidca.init()
	myMidca.run()
	baxter.disable_robot(True)

if __name__ == "__main__":
	test()
