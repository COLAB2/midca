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

- midca/domains/\<domain-name\>/\<domain-name\>.sim
	- Created by the PDDL_util.py function from Zoreh


### What needs updated
- [midca/setup.py](https://github.com/TonePoems/midca/blob/master/setup.py)
	- Add domain to packages list
		-Insert domain below 
			```python
			packages=['midca',
                'midca.domains',"
			```	