from midca import base
from midca.experimental.baxter import baxter_module, baxter

def print_func(s):
	print s

def baxter_midca():
	world = "Baxter World"
	myMidca = base.PhaseManager(world, display = print_func)
	for phase in ["Simulate", "Perceive", "Interpret", "Eval", "Intend", "Plan", "Act"]:
		myMidca.append_phase(phase)
	myMidca.append_module("Act", baxter_module.BaxterWave([], [(0.4, -0.2, 0.2), (0.5, -0.2, 0.4), (0.4, -0.2, 0.2)]))
	return myMidca

def test():
	myMidca = baxter_midca()
	myMidca.init()
	myMidca.run()
	baxter.disable_robot(True)

if __name__ == "__main__":
	test()
