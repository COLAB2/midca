# This file contains helpful functions for the nbeacons domain

import os
from MIDCA.modules._plan import pyhop
'''
 translate MIDCA init state to problem file in JSHOP 
 
'''  
def jshop_state_from_world(world, STATE_FILE, name = "state"):
#     thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
#     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problems.shp"
    
    f = open(STATE_FILE, 'w')
    f.write('\n')
    f.write('(defproblem log-ran-10-1 logistics')
    f.write(' (')
    
    for obj in world.objects.keys():
        if  world.objects[obj].type.name == "SAIRPLANE":
            f.write("(SAIRPLANE " + obj + ")\n")
        elif world.objects[obj].type.name == "AIRPLANE":
            f.write("(AIRPLANE " + obj + ")\n")
        
        elif world.objects[obj].type.name == "LOCATION":
            f.write("(LOCATION " + obj + ")\n")
        elif world.objects[obj].type.name == "CITY":
            f.write("(CITY " + obj + ")\n")
        elif world.objects[obj].type.name == "AIRPORT":
            f.write("(AIRPORT " + obj + ")\n")
        elif world.objects[obj].type.name == "PACKAGE":
            f.write("(PACKAGE " + obj + ")\n") 
              
    
    for atom in world.atoms:
        
        if atom.predicate.name == "airplane-at":
            f.write("(airplane-at " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
            
        elif atom.predicate.name == "sairplane-at":
            f.write("(sairplane-at " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
            
        elif atom.predicate.name == "NearBy":
            f.write("(NearBy " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
            
        elif atom.predicate.name == "truck-at":
            f.write("(truck-at " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
            
        elif atom.predicate.name == "truck-c":
            f.write("(TRUCK " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
               
        elif atom.predicate.name == "IN-CITY":
            f.write("(IN-CITY " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
        
        elif atom.predicate.name == "IN-CITY-A":
            f.write("(IN-CITY " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
        
        elif atom.predicate.name == "IN-CITY":
            f.write("(IN-CITY " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
        elif atom.predicate.name == "obj-at":
            f.write("(obj-at " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
    
    f.write(")\n")
    f.close()
    
'''
translate MIDCA goal to JSHOP tasks
'''    
    
def jshop_tasks_from_goals(goals,pyhopState, STATE_FILE):
#     thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
#     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problems.shp"
    f = open(STATE_FILE, 'a')
    
    alltasks = []
    blkgoals = pyhop.Goal("goals")
    blkgoals.pos = {}
    f.write(" ((achieve-goals (list\n")
    for goal in goals:
        #extract predicate
        if 'predicate' in goal.kwargs:
            predicate = str(goal.kwargs['predicate'])
        elif 'Predicate' in goal.kwargs:
            predicate = str(goal.kwargs['Predicate'])
        elif goal.args:
            predicate = str(goal.args[0])
        else:
            raise ValueError("Goal " + str(goal) + " does not translate to a valid pyhop task")
        args = [str(arg) for arg in goal.args]
        if args[0] == predicate:
            args.pop(0)
        if predicate == "obj-at":
            f.write("(obj-at " +  args[0] + " " +  args[1] + ")\n")
        
        
        else:
            raise Exception("No task corresponds to predicate " + predicate)
    f.write(" ))))")
    f.close()


def jshop2_state_from_world(world, STATE_FILE, name = "state"):
    thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
# #     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problem"
#     STATE_FILE = "C:/Users/Zohreh/git/MIDCA/modules/_plan/jShop/problem"
    f = open(STATE_FILE, 'w')
    f.write('\n')
    f.write("(defproblem problem logistics\n")
    f.write("(\n")
    
    for obj in world.objects.keys():

        
        if world.objects[obj].type.name == "AIRPORT":
            f.write("(AIRPORT " + obj + ")\n")
   
              
    
    for atom in world.atoms:
        
        if atom.predicate.name == "airplane-at":
            f.write("(airplane-at " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
        elif atom.predicate.name == "truck-c":
            f.write("(TRUCK " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
                    
        elif atom.predicate.name == "truck-at":
            f.write("(truck-at " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
            
        elif atom.predicate.name == "IN-CITY":
            f.write("(IN-CITY " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
                
        elif atom.predicate.name == "obj-at":
            f.write("(obj-at " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
    
    f.write(")\n")
    f.close()
    
'''
translate MIDCA goal to JSHOP tasks
'''    
    

def jshop2_tasks_from_goals(goals,pyhopState, STATE_FILE):
    thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
# #     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problem"
#     STATE_FILE = "C:/Users/Zohreh/git/MIDCA/modules/_plan/jShop/problem"
    f = open(STATE_FILE, 'a')
    
    alltasks = []
    f.write(" (:unordered\n")
    for goal in goals:
        #extract predicate
        if 'predicate' in goal.kwargs:
            predicate = str(goal.kwargs['predicate'])
        elif 'Predicate' in goal.kwargs:
            predicate = str(goal.kwargs['Predicate'])
        elif goal.args:
            predicate = str(goal.args[0])
        else:
            raise ValueError("Goal " + str(goal) + " does not translate to a valid pyhop task")
        args = [str(arg) for arg in goal.args]
        if args[0] == predicate:
            args.pop(0)
        if predicate == "obj-at":
            f.write("(obj-at " +  args[0] + " " +  args[1] + ")\n")
        
        
        else:
            raise Exception("No task corresponds to predicate " + predicate)
    f.write(" ))")
    f.close()
   
   