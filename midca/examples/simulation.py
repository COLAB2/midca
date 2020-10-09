#!/usr/bin/env python
from midca import base
from midca.modules import simulator
from midca.worldsim import domainread, stateread
import inspect, os

# Domain Specific Imports
from midca.domains.blocksworld import util
from midca.domains.blocksworld.plan import methods_multiAgent as methods, operators_multiAgent as operators


'''
Simulation of tower construction and arson prevention in blocksworld. Uses
TF-trees and simulated Meta-AQUA connection to autonomously generate goals.
'''

thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

MIDCA_ROOT = thisDir + "/../"

### Domain Specific Variables
DOMAIN_ROOT = MIDCA_ROOT + "domains/blocksworld/"
DOMAIN_FILE = DOMAIN_ROOT + "domains/arsonist_multiAgent.sim"
STATE_FILE = DOMAIN_ROOT + "states/defstate_multiAgent.sim"
DISPLAY_FUNC = util.asqiiDisplay

world = domainread.load_domain(DOMAIN_FILE)
stateread.apply_state_file(world, STATE_FILE)
#creates a PhaseManager object, which wraps a MIDCA object
myMidca = base.PhaseManager(world, display = DISPLAY_FUNC, verbose=4)
#add phases by name
for phase in ["Simulate"]:
    myMidca.append_phase(phase)

publish = "tcp://127.0.0.1:6000"
subscribe = "tcp://127.0.0.1:7000"

#add the modules which instantiate basic blocksworld operation
myMidca.append_module("Simulate", simulator.RemoteMidcaActionSimulator(publish, subscribe))
myMidca.append_module("Simulate", simulator.SendWorld(publish, subscribe))
myMidca.append_module("Simulate", simulator.ASCIIWorldViewer(display=DISPLAY_FUNC))


#tells the PhaseManager to copy and store MIDCA states so they can be accessed later.
myMidca.storeHistory = True
myMidca.init()
myMidca.run(False, phaseDelay=0)

'''
The code below would print out MIDCA's goal set for the first 20 phases of the run above. Note that any memory values can be accessed in this way, assuming that the storeHistory value was set to True during the run. This code is left as an example, but commented out because it will throw an error if fewer than 20 cycles were simulated.

for i in range(20):
    print myMidca.history[i].mem.get(myMidca.midca.mem.CURRENT_GOALS)

'''
