'''
This file controls option management in MIDCA. By default, options are used to control several parameters and run options, and can be customized at startup using a text interface. 

To add completely new options, however, this file must be modified. Specifically, the "all_options" method must be modified to return the desired option list. Each item in that list should be of the Option class, as defined below. The constructor for an Option takes the following parameters:

name -> this is how the option will be referenced everywhere. Do not duplicate names.

queryType -> Choices currently include "bool" and "num". This affects how users are asked to select options and what values they can take.

queryText -> What text will be displayed to the user. Additionally, the default or current value will be displayed.

default -> default value for this option.

dependsOn -> other option (referenced by name) that this option depends on. For example, the "random-fires-chance" option lists "random-fires" as its "dependsOn" values, and thus will only be queried if the "random-fires" option is enabled. "Enabled" is defined as user-selected (not default) and non-false. 
'''

import sys
sys.path.append("../")
import cPickle

def bool_query(text):
	ans = raw_input(text + " y/n")
	while not (ans.startswith("y") or ans.startswith("n") or ans == ""):
		ans = raw_input("Please answer with y or n")
	if ans == "":
		return "def"
	return ans.startswith("y")

def is_number(s):
	try:
		float(s)
		return True
	except ValueError:
		return False

def num_query(text):
	ans = raw_input(text)
	while not (is_number(ans) or ans == ""):
		ans = raw_input("Please answer with a number")
	if ans == "":
		return "def"
	return float(ans)



class Option:
	
	def __init__(self, name, queryType, queryText, default, dependsOn = None):
		self.name = name
		self.queryType = queryType
		self.queryText = queryText
		self.res = default
		self.dependsOn = dependsOn
		self.answered = False
	
	def applicable(self, valDict):
		return not self.dependsOn or (self.dependsOn in valDict and valDict[self.dependsOn])
	
	def ask(self):
		if self.queryType == "bool":
			res = bool_query(self.queryText + " (" + str(self.res) + ")")
			if res != "def":
				self.res = res
				self.answered = True
		elif self.queryType == "num":
			res = num_query(self.queryText + " (" + str(self.res) + ")")
			if res != "def":
				self.res = res
				self.answered = True
	
	def apply(self, valDict):
		valDict[self.name] = self.res

class OptionSet:
	
	def __init__(self):
		self.staticOptions = {}
		self.customOptions = {}
	
	def add_option(self, option, static):
		if static:
			if option.name in self.staticOptions:
				add = bool_query("option already exists. Replace old value?", True)
				if not add:
					return
			self.staticOptions[option.name] = option
		else:
			if option.name in self.customOptions:
				add = bool_query("option already exists. Replace old value?", True)
				if not add:
					return
			self.customOptions[option.name] = option
		
	def get_values(self, staticOnly = False):
		valDict = {}
		for option in self.staticOptions.values():
			option.apply(valDict)
		if not staticOnly:
			for option in self.customOptions.values():
				option.apply(valDict)
		return valDict
	
	def ask_all_custom(self):
		valDict = self.get_values(staticOnly = True)
		while self.customOptions:
			answered = []
			for option in self.customOptions.values():
				if option.applicable(valDict):
					option.ask()
					if option.answered:
						option.apply(valDict)
						answered.append(option)
			if not answered:
				break
			for option in answered:
				del self.customOptions[option.name]
				self.staticOptions[option.name] = option
	
	def get(self, name):
		if name in self.staticOptions:
			return self.staticOptions[name].res
		elif name in self.customOptions:
			return self.customOptions[name].res
		return None

def all_options():
	options = []
	
	#a dist exists?
	options.append(Option("is-ADist", "bool", "Track A-Distance in note phase?", True))
	#a dist params
	options.append(Option("ADist-window-size", "num", "A-distance window size?", 10, "is-ADist"))
	options.append(Option("ADist-threshold", "num", "A-distance threshold/epsilon?", 0.3, "is-ADist"))
	
	#put out fires on tower completion?
	options.append(Option("restart-fires", "bool", "Put out fires when a tower is completed?", True))
	
	#use GNG?
	options.append(Option("GNG", "bool", "Run GNG on ADist results?", False, "is-ADist"))
	
	#Usage of TF-Trees and Meta-AQUA
	options.append(Option("TF-base", "bool", "Use TF-Trees to generate block stacking goals?", True))
	options.append(Option("TF-Fire", "bool", "Use TF-Trees to generate fire removal goals?", True))
	options.append(Option("catch-arsonist", "bool", "Generate arsonist catching goals?", True))
	options.append(Option("MA-arson", "bool", "Use Meta-AQUA to generate arsonist catching goals?", False))
	
	#arsonist
	options.append(Option("arsonist", "bool", "Is there an arsonist?", True))
	options.append(Option("arsonist-start", "num", "At what time step does the arsonist start setting fires?", 10, "arsonist"))
	options.append(Option("arsonist-chance", "num", "What is the chance that the arsonist will set a fire at each time step after he is activated?", 0.5, "arsonist"))
	options.append(Option("arsonist-long-apprehend", "bool", "Will the arsonist take multiple steps to capture?", False, "arsonist"))
	
	#random fires
	options.append(Option("random-fires", "bool", "Will random fires occur", False))
	options.append(Option("random-fires-start", "num", "At what time step do random fires start occuring?", 10, "random-fires"))
	options.append(Option("random-fires-chance", "num", "What is the chance that random-fires will occur at each time step after they are activated?", 0.2, "random-fires"))
	
	#planning
	options.append(Option("plan-on-complete", "bool", "Will new goals only be sent to the planner when there is no current plan?", True))
	options.append(Option("override-for-priority", "bool", "Will old plans be overriden by higher priority goals?", True, "plan-on-complete"))
	options.append(Option("prioritize-new-goals", "bool", "Should goals generated more recently be prioritized?", True))
	
	return options
	
	
def custom_setup(optionSet = None):
	if not optionSet:
		optionSet = OptionSet()
		for option in all_options():
			optionSet.add_option(option, False)
	print "starting custom setup. Leave any response blank to skip.\n"
	
	optionSet.ask_all_custom()
	return optionSet
	

SAVE_DIR = "./custom/option/save/"
SAVE_EXT = ".opt"

def load_setup(name):
	#try:
	f = open(SAVE_DIR + name + SAVE_EXT)
	optionSet = cPickle.load(f)
	return optionSet
	#except Exception:
	#return None		

def save_setup(optionSet, name):
	try:
		f = open(SAVE_DIR + name + SAVE_EXT, 'w')
		cPickle.dump(optionSet, f)
		return True
	except Exception:
		return False
	