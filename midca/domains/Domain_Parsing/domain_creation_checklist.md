# Domain Creation

The script has been started and is called New_Domain_Script.py and is found in midca/domains
(C:\Users\Anthony\Anaconda3\python.exe New_Domain_Script.py minecraft/minecraft.pddl)

### What needs to happen in MIDCA when a new domain is created.
This file should act as a master list of files that need updated when a new domain is added, not necessarily a walkthrough for new users.

https://github.com/COLAB2/midca/wiki/How-to-add-a-new-domain
tmp: https://github.com/TonePoems/midca/blob/zohreh-minecraft-metricff/midca/domains/minecraft/PDDL_util.py
tmp: https://github.com/COLAB2/midca/tree/zohreh-minecraft-metricff/midca/domains/ffdomain/minecraft

### What needs to be created
- midca/domains/\<domain-name\>/init.py	
	-blank file needed for MIDCA

- midca/examples/\<domain-name\>.py
	- Based off of a template currently being created
		- examples_template.txt
		
		- Needs to regex replace "\<domain-name\>" within the template with the new domain name
		
		- Based on 
		- https://github.com/COLAB2/midca/blob/master/midca/examples/nbeacons_demo.py
		- https://github.com/COLAB2/midca/blob/master/midca/examples/simple_run.py
		- https://github.com/COLAB2/midca/blob/master/midca/examples/chicken_run.py
	
	- Needs to update the README.txt in the same directory

- midca/domains/\<domain-name\>/\<domain-name\>.sim
	- Created by the PDDL_util.py function from Zoreh

- midca/domains/\<domain-name\>/util.py
	- Based off of a template currently being created
		- util_template.txt
		
		- Needs to regex replace "\<domain-name\>" within the template with the new domain name

- midca/domains/\<domain-name\>/plan/methods.py
	- Based off of a template currently being created
		- methods_template.txt
		
		- Needs to regex replace "\<domain-name\>" within the template with the new domain name
		
		- Based on 
		- https://github.com/COLAB2/midca/blob/931a0430f72083227f952e0cb57f445c15e51548/midca/domains/nbeacons/plan/methods_nbeacons.py#L126

- midca/domains/\<domain-name\>/plan/operators.py
	- Based off of a template currently being created
		- operators_template.txt
		
		- Needs to regex replace "\<domain-name\>" within the template with the new domain name
		
		- Based on 
		- https://github.com/COLAB2/midca/blob/931a0430f72083227f952e0cb57f445c15e51548/midca/domains/blocksworld/plan/operators.py#L81

### What needs updated
- [midca/setup.py](https://github.com/TonePoems/midca/blob/master/setup.py)
	- Add domain to packages list
	
	- Add any new file types to package_data

- Midca/modules with domain specific functions
	```
	# TODO: Update the module references to be domain specific functions
	# Add the modules which instantiate basic operation
	myMidca.append_module("Simulate", simulator.MidcaActionSimulator())
	myMidca.append_module("Perceive", perceive.PerfectObserver())
	myMidca.append_module("Interpret", guide.UserGoalInput())
	myMidca.append_module("Eval", evaluate.SimpleEval())
	myMidca.append_module("Intend", intend.SimpleIntend())
	myMidca.append_module("Plan", planning.GenericPyhopPlanner(
		DECLARE_METHODS_FUNC, DECLARE_OPERATORS_FUNC)) # set up planner for sample domain
	myMidca.append_module("Act", act.SimpleAct())
	```

	-	[midca/modules/simulator.py](https://github.com/TonePoems/midca/blob/master/midca/modules/simulator.py)
		- simulator_template.txt will be added to the .py file
	
		- Reference by `myMidca.append_module("Simulate", simulator.<domain-name>Simulator())` in midca/examples/\<domain-name\>.py

	-	[midca/modules/guide.py](https://github.com/TonePoems/midca/blob/master/midca/modules/guide.py)
		- 
	
		- Reference by `myMidca.append_module("Interpret", guide.<domain-name>Goal())` in midca/examples/\<domain-name\>.py
		
		- Goals need to come from a state file for the domain

		- Add class for goal generator
		
		- Can use UserGoalInput for any domain

	-	[midca/modules/evaluate.py](https://github.com/TonePoems/midca/blob/master/midca/modules/evaluate.py)
		- 
	
		- Reference by `myMidca.append_module("Eval", evaluate.<domain-name>Eval())` in midca/examples/\<domain-name\>.py

	-	[midca/modules/intend.py](https://github.com/TonePoems/midca/blob/master/midca/modules/intend.py)
		- 
	
		- Reference by `myMidca.append_module("Intend", intend.<domain-name>Intend())` in midca/examples/\<domain-name\>.py

	-	[midca/modules/planning.py](https://github.com/TonePoems/midca/blob/master/midca/modules/planning.py)
		- 
	
		- Reference by `myMidca.append_module("Plan", planning.GenericPyhopPlanner(DECLARE_METHODS_FUNC, DECLARE_OPERATORS_FUNC))` in midca/examples/\<domain-name\>.py

	-	[midca/modules/act.py](https://github.com/TonePoems/midca/blob/master/midca/modules/act.py)
		- 
	
		- Reference by `myMidca.append_module("Act", act.<domain-name>Act())` in midca/examples/\<domain-name\>.py
