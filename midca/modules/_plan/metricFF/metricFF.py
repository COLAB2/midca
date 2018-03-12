import inspect, os
import subprocess
import re
from subprocess import Popen, PIPE, STDOUT

#this works with linux system

def metricFF(tasks, DOMAIN_FIILE, STATE_FILE):
  
    thisDir =  os.path.dirname(os.path.realpath(__file__))
    MIDCA_ROOT = thisDir + "/../../../"
    cwd = os.getcwd()
    os.chdir(thisDir)

    command = './metric-ff' + ' -o ' + DOMAIN_FIILE + ' -f ' + STATE_FILE
    
#     process = subprocess.Popen([command], stdout=subprocess.PIPE)
    p = subprocess.Popen([command], shell=True, stdout=subprocess.PIPE)
    out, err = p.communicate()
    print out
    plan = []
    lines = out.split("\n")
    for line in lines:
        line = line.strip()
        if line.startswith("step"):
            plan.append((line.split(":")[1]).strip())
        if re.match("[0-9]*:.*", line):
            plan.append((line.split(":")[1]).strip())
    
    
#     pipe = os.popen (command)
#     lstFFOutput = pipe.readlines ()
#     print lstFFOutput
#     pipe.close ()
    
#     for line in p.stdout:
#         print line
        

            
if __name__ == "__main__": 
    
    thisDir =  os.path.dirname(os.path.realpath(__file__))
    MIDCA_ROOT = thisDir + "/../../../"
    
    DOMAIN_FIILE =  MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
#     #DOMAIN_FIILE = JSHOP_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
    STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood-pickaxe.97.pddl"
    
    metricFF("tasks", DOMAIN_FIILE, STATE_FILE)
            
            
            