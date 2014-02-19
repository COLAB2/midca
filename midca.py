import phasemanager, sys
from custom import customsetup, memconstants
from custom.option import options, setup

def run(midca):
	while 1:
		val = raw_input()
		if val == "q":
			if hasattr(customsetup, "custom_cleanup"):
				customsetup.custom_cleanup(midca)
			break
		elif val == "skip":
			midca.one_cycle(verbose = 0, pause = 0)
			print "cycle finished"
		elif val == "show":
			print midca.get_module("Observation").world_repr(midca.world)
			print str(midca.world)
		elif val.startswith("skip"):
			try:
				num = int(val[4:].strip())
				for i in range(num):
					midca.one_cycle(verbose = 0, pause = 0)
				print str(num) + " cycles finished."
			except ValueError:
				print "Usage: 'skip n', where n is an integer"
		elif val == "adist":
			for i in range(100):
				midca.one_cycle(verbose = 0, pause = 0)
				if i % 10 == 0:
					print i, ": ", midca.mem.get(memconstants.MEM_ANOM)
			print midca.mem.get(memconstants.MEM_ANOM)
		elif val == "?" or val == "help":
			print "interface: \n enter/return -> input commands. Empty command goes to next cycle \n q -> quit \n skip n -> skips n cycles \n show -> print world representation \n ? or help -> show this list of commands \n adist -> run aDist test. This may not work in current version.\n"
		else:
			midca.next_phase()

def init():
	choice = raw_input("Type the name of the configuration you would like to use with MIDCA, or press enter to customize a new one. Press q to quit. Option setups are located in the setup/option/save directory, and should be referenced without the \".opt\" ending.")
	loading = False
	if choice != "":
		if choice == "q":
			return
		optionSet = options.load_setup(choice)
		if not optionSet:
			print "error loading options"
			return init()
		else:
			optionSet = options.custom_setup(optionSet)
			print "option set loaded successfully."
			loading = True
	else:
		optionSet = options.custom_setup()
	optionSet.ask_all_custom()
	
	#only save if a new custom set was created.
	if not loading:
		saveQ = options.bool_query("Done with customization. Save results?")
		if saveQ:
			name = raw_input("what is the name of this option set?")
			while name == "":
				name = raw_input("what is the name of this option set? Enter q to cancel save.")
			if name != 'q':
				worked = options.save_setup(optionSet, name)
				if worked:
					print "options saved successfully"
				else:
					print "option save failed."
	
	midca = setup.initialize(optionSet)
	return midca
	

if __name__ == "__main__":
	midca = init()
	if not midca:
		sys.exit()
	run(midca)
	
	