# Domain Creation

### What needs to happen in MIDCA when a new domain is created.
This file should act as a master list of files that need updated when a new domain is added, not necessarily a walkthrough for new users.

https://github.com/COLAB2/midca/wiki/How-to-add-a-new-domain
tmp: https://github.com/TonePoems/midca/blob/zohreh-minecraft-metricff/midca/domains/minecraft/PDDL_util.py
tmp: https://github.com/COLAB2/midca/tree/zohreh-minecraft-metricff/midca/domains/ffdomain/minecraft

### What needs to be created
- midca\examples\<domain-name>.py
	- Based off of a template currently being created
		- examples_template.txt
		
		- Needs to regex replace "\<domain-name\>" within the template with the new domain name
		
		- Based on 
		- https://github.com/COLAB2/midca/blob/master/midca/examples/nbeacons_demo.py
		- https://github.com/COLAB2/midca/blob/master/midca/examples/simple_run.py
		- https://github.com/COLAB2/midca/blob/master/midca/examples/chicken_run.py
	
	- Needs to update the README.txt in the same directory

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
- midca\setup.py
	- Add domain to packages list
	
	- Add any new file types to package_data

- most likely have to update all of the midca\modules with a domain specific function
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



- midca\modules\guide.py
	- Goals need to come from a state file for the domain

	- Add class for goal generator
	
	- Can use UserGoalInput for any domain
	
	- Used in the examples\<domain-name>.py for the interpret module being appended or inserted

