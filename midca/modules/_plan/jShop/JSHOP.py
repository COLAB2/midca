import inspect, os
import subprocess
import time

from subprocess import Popen, PIPE, STDOUT


def jshop(tasks, DOMAIN_FIILE, STATE_FILE, verbose=1):

    thisDir =  os.path.dirname(os.path.realpath(__file__))
#     thisDir = "C:/Users/Zohreh/git/midca/modules/_plan/jShop/"
    MIDCA_ROOT = thisDir + "/../../../"

#   DOMAIN_FIILE = MIDCA_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
#     #DOMAIN_FIILE = JSHOP_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
#    STATE_FILE = MIDCA_ROOT + "domains/jshop_domains/blocks_world/bw_ran_problems_5.shp"
#
#     f = open(STATE_FILE, 'r')
#     a = f.read()
#     print a
#

    p = Popen(['java', '-jar', thisDir+'/jshop.jar', DOMAIN_FIILE,
               STATE_FILE, '1'], stdout=PIPE, stderr=STDOUT)


    for line in p.stdout:
        if verbose > 1:
            #time.sleep(1)
            print line
        if(line.startswith(" ( (!")):
            plan = line
            break

    if(plan):
        Jshop_plan = graceParse(parse(plan))

    return Jshop_plan

def Tile(coordinates):
    return "Tx" + str(int(float(coordinates[0]))) + "y" + str(int(float(coordinates[1])))

def graceParse(plan):
    """

    :return: convert grid coordinates to tile co-ordinates
    """
    for i,each in enumerate(plan):
        if each[0].startswith("move") and not each[0] == "moveto":
            tile1 = Tile(each[2:4])
            tile2 = Tile(each[4:])
            remaining = each[6:]
            each = each[:2]
            each.append(tile1)
            each.append(tile2)
            if remaining:
                for arg in remaining:
                    each.append(arg)
            plan[i] = each

        elif each[0] == "moveto":
            tile1 = Tile(each[4:6])
            each = each[:4]
            each.append(tile1)
            plan[i] = each

        elif each[0] == "ascend" or each[0] == "descend":
            tile1 = Tile(each[2:4])
            each = each[:2]
            each.append(tile1)
            plan[i] = each

        elif each[0].startswith("collect") or each[0].startswith("deepcollect"):
            try:
                tile2 = Tile(each[3:])
                each = each[:3]
            except:
                tile2 = Tile(each[4:])
                each = each[:4]
            each.append(tile2)
            plan[i] = each

        elif each[0].startswith("communicate"):
            try:
                tile2 = Tile(each[4:])
                each = each[:4]
            except:
                tile2 = Tile(each[5:])
                each = each[:5]
            each.append(tile2)
            plan[i] = each
    return plan



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
            action_list = elm.strip().split(' ')
            plan.append(action_list)

    return plan

if __name__ == "__main__":
    jshop("tasks", "/home/sampath/Documents/git/midca/midca/domains/grace/plan/midcaNsf.shp",
    "/home/sampath/Documents/git/midca/midca/domains/grace/plan/midcansfproblem.shp", 2)
