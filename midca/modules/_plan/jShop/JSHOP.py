import inspect, os
import subprocess

from subprocess import Popen, PIPE, STDOUT

def jshop(tasks, DOMAIN_FIILE, STATE_FILE):
  
    thisDir =  os.path.dirname(os.path.realpath(__file__))
#     thisDir = "C:/Users/Zohreh/git/midca/modules/_plan/jShop/"
    MIDCA_ROOT = thisDir + "/../../../"
    
#     DOMAIN_FIILE = MIDCA_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
#     #DOMAIN_FIILE = JSHOP_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
#     STATE_FILE = MIDCA_ROOT + "domains/jshop_domains/blocks_world/bw_ran_problems_5.shp"
#     
#     f = open(STATE_FILE, 'r')
#     a = f.read()
#     print a
#   
    
    p = Popen(['java', '-jar', thisDir+'/jshop.jar', DOMAIN_FIILE,
               STATE_FILE, '1'], stdout=PIPE, stderr=STDOUT)
    
    for line in p.stdout:
        print line
        if(line.startswith(" ( (!")):
            plan = line
            break
    
    if(plan):   
        Jshop_plan = parse(plan)
    
    return Jshop_plan

def parenthetic_contents(string):
    """Generate parenthesized contents in string as pairs (level, contents)."""
    stack = []
    for i, c in enumerate(string):
        if c == '(':
            stack.append(i)
        elif c == ')' and stack:
            start = stack.pop()
            yield (string[start + 1: i])

def parse(str):
    elements  = list(parenthetic_contents(str))
    plan = []
    for elm in elements:
        if(elm[0] == '!' and '(' not in elm):
            elm = elm[1:]
            print elm
            action_list = elm.strip().split(' ')
            plan.append(action_list)
    
    return plan        
            
if __name__ == "__main__": 
    jshop("tasks")
            
            
            