#this one uses moveit to move Baxter' arm

from midca import base
from midca.experimental.baxter import moveit_test, baxter

def print_func(s):
	print s

def baxter_midca():
	world = "Baxter World"
	print 'I am here'	
	myMidca = base.PhaseManager(world, display = print_func)
	for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)
	
	
	myMidca.append_module("Act", moveit_test.BaxterWave([], [(0.5212523750330741, 0.32016779873003876,-0.06583053722449383)]))
	return myMidca

def test():
	myMidca = baxter_midca()
	myMidca.init()
	myMidca.run()
	baxter.disable_robot(True)

if __name__ == "__main__":
	test()
