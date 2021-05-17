import os,sys
from contextlib import contextmanager
import time,subprocess

subprocess.run(["python3","fast_downward.py","plant-protection-modif.pddl","plant-pro-modif.pddl","--search","lazy_greedy([ff])"])


try:
#self.mem.set(self.mem.PLANNING_COUNT, 1 + self.mem.get(self.mem.PLANNING_COUNT))
#ffPlan = metricFF.metric_ff(self.domain_file, self.state_file)
	
	ffplan = os.system('python3 fast_downward.py plant-protection-modif.pddl plant-pro_modif.pddl --search "lazy_greedy([ff])"')
	print(ffplan)
	with open('sas_plan') as f:
		for line in f:
			print(line.strip())
except Exception:
	ffPlan = None
	#if not ffPlan and ffPlan != []:
	#	if verbose >= 1:
	print("Planning failed for ")



