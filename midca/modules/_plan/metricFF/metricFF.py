import inspect, os
import subprocess
import re
from subprocess import Popen, PIPE, STDOUT


# this works with linux system

def metric_ff(DOMAIN_FIILE, STATE_FILE):
    plan = [['move', 'm0_0', 'm0_1'], ['get-harvest-wood', 'm0_1', 'hand']]
    return plan


def metric_ff2(DOMAIN_FIILE, STATE_FILE):
    thisDir = os.path.dirname(os.path.realpath(__file__))
    MIDCA_ROOT = thisDir + "/../../../"
    cwd = os.getcwd()
    os.chdir(thisDir)

    command = './metric-ff' + ' -o ' + DOMAIN_FIILE + ' -f ' + STATE_FILE

    #     process = subprocess.Popen([command], stdout=subprocess.PIPE)
    p = subprocess.Popen([command], shell=True, stdout=subprocess.PIPE)
    out, err = p.communicate()
    print
    out
    plan = []
    lines = out.split("\n")
    for line in lines:
        line = line.strip()
        if line.startswith("step"):
            plan.append((line.split(":")[1].split(" ")))
        if re.match("[0-9]*:.*", line):
            plan.append((line.split(":")[1].split(" ")))
    return plan


if __name__ == "__main__":
    thisDir = os.path.dirname(os.path.realpath(__file__))
    MIDCA_ROOT = thisDir + "/../../../"

    DOMAIN_FIILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
    #     #DOMAIN_FIILE = JSHOP_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
    STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood-pickaxe.97.pddl"

    metric_ff(DOMAIN_FIILE, STATE_FILE)
