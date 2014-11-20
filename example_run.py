
from midca.examples import configurations

#instantiates a preset version of MIDCA with basic versions of several phases. 
myMidca = configurations.SimplePredicateMIDCA(myDomainFile, myStateFile)

from midca.modules import note

#add a module in the interpret phase to perform A-distance and check for anomalies.
myMidca.append_module(phase = "interpret", 
					  module = note.ADistanceAnomalyNoter(threshold = 0.3))


myMidca.init() #initialize modules
myMidca.run() #run using text-based interface as in the past

#write anomaly detection results to file
f = open("./anomaly_results.txt", 'w')
f.write(myMidca.mem.get("anomaly state"))

myMidca.reset() #reset MIDCA, i.e. wipe memory and reset cycle count
myMidca.setWorldState(myState) 
#myState is a state file or actual world state object

myMidca.init() #initialize modules
myMidca.run() #run using text-based interface as in the past
...

