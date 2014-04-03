'''
To employ user-defined modules, customize the get_modules() method. This method takes a list of user-defined options as an input, allowing flexible customization. Each module should be a class with an "init(world, memory)" method and a "run(verbose)" method. The method should output a dictionary of {phaseName: object}, where "object" is an instance of the class with the init/run methods.

To use different phases, modify the PHASES variable. Note that any phase without a module assigned in get_modules will be skipped, and the order of phases will be that given by the PHASES variable, not in module assignment.

To use a different domain or state file, modify the DOMAIN_F and STATE_F variables. 

To perform a custom initialization (following MIDCA's usual setup), modify the custom_init() method below. This method must take one parameter, the midca instance that has just been created.
'''
import sys, time
sys.path.append("../")
import socket
from modules import *
from modules import dummy
from worldsim import domainread, stateread

PHASES = ["Observation", "Note", "Evaluation", "Assess", "Goal Selection", "Intend", "Planning", "Action Selection"]

SOCKET_R_PORT = 5156
SOCKET_W_PORT = 5151
SOCKET_WAIT = 3 #wait for MA before attempting read connection

DOMAIN_F = "./worldsim/domains/arsonist.sim"
STATE_F = "./worldsim/states/defstate.sim"

#defines module that simulates the world MIDCA operates in. Must have same methods as a normal module, see top of file for details.
def get_world_sim(options):
	#world setup
	if not options.get("random-fires"):
		maxFires = 0
	else:
		maxFires = 1000000
	return simulator.Simulator(isArsonist = options.get("arsonist"), arsonChance = options.get("arsonist-chance"), arsonStart = options.get("arsonist-start"), firechance = options.get("random-fires-chance"), firestart = options.get("random-fires-start"), maxRandomFires = maxFires)

#this method defines the modules that constitute MIDCA's phases. See top of file for details.
def get_modules(options):
		
	modules = {}
	
	#observe phase
	modules["Observation"] = observe.observe.SimpleObserver()
	
	#note phase
	if options.get("is-ADist"):
		windowSize = options.get("ADist-window-size")
		threshold = options.get("ADist-threshold")
		modules["Note"] = note.note.ADNoter(windowSize, threshold)
	
	#eval phase
	modules["Evaluation"] = evaluate.evaluate.Evaluator(options.get("restart-fires"))
	
	#assess
	if options.get("GNG"):
		modules["Assess"] = assess.assess.Assessor(windowSize, threshold, options.get("Valence"))
	
	#goal selection
	modules["Goal Selection"] = goalinsertion.goalinsert.NewGuide(options.get("TF-base"), options.get("TF-Fire"), options.get("catch-arsonist"), options.get("MA-arson"), options.get("prioritize-new-goals"))
	
	#intend
	modules["Intend"] = intend.intend.SimpleIntend(options.get("plan-on-complete"), options.get("override-for-priority"))
	
	#planning
	modules["Planning"] = plan.pyplan.PyHopPlanner(options.get("arsonist-long-apprehend"))
	
	#execution
	modules["Action Selection"] = execute.execute.Exec()
	
	#dummy, testing.
	modules["Dummy"] = dummy.Dummy()
	
	return modules

#reads domain and world state from files specified above.	
def read_world():
	world = domainread.load_domain(DOMAIN_F)
	stateread.apply_state_file(world, STATE_F)
	return world
	
#connects to Meta-AQUA through sockets	
def setup_sockets(midca):
	print "starting setup"
	sWrite = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sWrite.connect(("localhost", SOCKET_W_PORT))
	midca.mem._update(midca.memKeys.SOCKET_W, sWrite)
	time.sleep(SOCKET_WAIT)
	print "write"
	sRead = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sRead.connect(("localhost", SOCKET_R_PORT))
	midca.mem._update(midca.memKeys.SOCKET_R, sRead)
	print "setup complete"

#closes connection to Meta-AQUA through sockets	
def close_sockets(midca):
	writeSocket = midca.mem.get(midca.memKeys.SOCKET_W)
	writeSocket.send("Done")
	readSocket = midca.mem.get(midca.memKeys.SOCKET_R)
	writeSocket.shutdown(socket.SHUT_RDWR)
	writeSocket.close()
	readSocket.shutdown(socket.SHUT_RDWR)
	readSocket.close()

#user-defined setup
def custom_init(midca):
	if midca.options.get("MA-arson"):
		setup_sockets(midca)

def custom_cleanup(midca):
	if midca.options.get("MA-arson"):
		close_sockets(midca)

