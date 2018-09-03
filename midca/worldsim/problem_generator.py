# source: https://github.com/hfoffani/pddl-lib
# https://github.com/karpase/pythonpddl
#
# I ran this using Python 3.6 and ubuntu
# this script might not work on windows.

# install pip3
# python3.6 -m pip install


#
# pip3 install antlr4-python3-runtime
# clone https://github.com/karpase/pythonpddl
# follow the steps in the the link above

import midca.worldsim.worldsim as worldsim

import inspect, os, random

SKELETON_CHANCE = 0.8
ARROW_TRAP_CHANCE = 0.8
# MONSTER_CHANCE = 0.3
SKELETON_LOC = 0
TRAP_LOC = 0
MONSTER_LOC = 0
n = 10

# Random Problem Generator
def generate_file(file):
    f = open(file, "w")

    f.write("(define \n")
    f.write("(problem wood)\n")
    f.write("(:domain minecraft-beta)\n")
    f.write("(:objects\n")

    for i in range(n):
        for j in range(n):
            f.write("m" + str(i) + "_" + str(j) + " - mapgrid\n")

    f.write("tree - resource\n")

    f.write("g11 g12 g13 g21 g22 g23 g31 g32 g33 - craftgrid\n")
    f.write("shelter - resource\n")
    f.write("skeleton - resource\n")
    f.write("arrowtrap - resource\n")
    f.write("arrow - resource\n")
    f.write("bone - resource\n")
    # f.write("monster - resource\n")
    f.write("monster-remains - resource\n")
    f.write("helmet - helmet\n")
    f.write("chestplate - chestplate\n")
    f.write("instant-health-potion - potion\n")
    f.write("wood - resource\n")
    f.write("wood-axe - tool\n")
    f.write("hand - tool\n")
    f.write("bow - tool\n")
    f.write("bowl - material")
    f.write(")\n")

    f.write("(:init\n")

    for i in range(n):
        for j in range(n):
            if i + 1 < n:
                f.write("(connect m" + str(i) + "_" + str(j) + " " + "m" + str(i + 1) + "_" + str(j) + ")\n")
            if i - 1 >= 0:
                f.write("(connect m" + str(i) + "_" + str(j) + " " + "m" + str(i - 1) + "_" + str(j) + ")\n")
            if j + 1 < n:
                f.write("(connect m" + str(i) + "_" + str(j) + " " + "m" + str(i) + "_" + str(j + 1) + ")\n")
            if j - 1 >= 0:
                f.write("(connect m" + str(i) + "_" + str(j) + " " + "m" + str(i) + "_" + str(j - 1) + ")\n")

    for i in range(n):
        for j in range(n):
            if i + 1 < n and j+1 < n:
                f.write("(connect-right m" + str(i) + "_" + str(j) + " " + "m" + str(i + 1) + "_" + str(j+1) + ")\n")
            if i - 1 >= 0 and j+1 < n:
                f.write("(connect-right m" + str(i) + "_" + str(j) + " " + "m" + str(i - 1) + "_" + str(j+1) + ")\n")
            if j + 1 < n:
                f.write("(connect-right m" + str(i) + "_" + str(j) + " " + "m" + str(i) + "_" + str(j + 1) + ")\n")

            if i + 1 < n and j -1 >= 0:
                f.write("(connect-left m" + str(i) + "_" + str(j) + " " + "m" + str(i + 1) + "_" + str(j -1) + ")\n")
            if i - 1 >= 0 and j -1 >= 0:
                f.write("(connect-left m" + str(i) + "_" + str(j) + " " + "m" + str(i - 1) + "_" + str(j -1) + ")\n")
            if j -1 >= 0:
                f.write("(connect-left m" + str(i) + "_" + str(j) + " " + "m" + str(i) + "_" + str(j  -1) + ")\n")

            if i + 1 < n and j - 1 >= 0:
                f.write("(connect-behind m" + str(i) + "_" + str(j) + " " + "m" + str(i + 1) + "_" + str(j - 1) + ")\n")
            if i + 1 < n:
                f.write("(connect-behind m" + str(i) + "_" + str(j) + " " + "m" + str(i + 1) + "_" + str(j) + ")\n")
            if i + 1 < n and j + 1 < n:
                f.write("(connect-behind m" + str(i) + "_" + str(j) + " " + "m" + str(i + 1) + "_" + str(j + 1) + ")\n")

            if i - 1 >= 0 and j - 1 >= 0:
                f.write("(connect-forward m" + str(i) + "_" + str(j) + " " + "m" + str(i - 1) + "_" + str(j - 1) + ")\n")
            if i - 1 >= 0:
                f.write("(connect-forward m" + str(i) + "_" + str(j) + " " + "m" + str(i - 1) + "_" + str(j) + ")\n")
            if i - 1 >= 0 and j + 1 < n:
                f.write("(connect-forward m" + str(i) + "_" + str(j) + " " + "m" + str(i - 1) + "_" + str(j + 1) + ")\n")

    f.write(" (player-at m0_0)\n")

    f.write("(= (player-current-health) 20)\n")

    f.write(" (= (tool-in-hand) 11)\n")
    f.write("(= (tool-id hand) 0)\n")
    f.write("(= (tool-id wood-axe) 11)\n")
    f.write("(= (tool-id bow) 10)\n")
    f.write("(= (current-harvest-duration) 0)\n")
    f.write("(= (current-harvest-location) 0)\n")

    f.write("(= (thing-available wood) 0)\n")
    f.write("(= (thing-available wood-axe) 1)\n")
    f.write("(= (thing-available bow) 1)\n")
    f.write("(= (thing-available instant-health-potion) 2)\n")
    f.write("(= (thing-available helmet) 1)\n")
    f.write("(= (thing-available chestplate) 1)\n")
    f.write("(= (thing-available bone) 0)\n")

    f.write("(= (duration-need hand tree) 1)\n")
    f.write("(= (duration-need wood-axe tree) 1)\n")
    f.write("(= (duration-need wood-axe skeleton) 1000000000)\n")
    f.write("(= (duration-need bow skeleton) 1000000000)\n")
    f.write("(= (duration-need wood-axe arrowtrap) 1000000000)\n")
    # f.write("(= (duration-need wood-axe monster) 1000000000)\n")
    f.write("(= (tool-max-health hand) 1000000000)\n")
    f.write("(= (tool-max-health wood-axe) 1000000000)\n")
    f.write("(= (tool-max-health bow) 1000000000)\n")
    f.write("(= (tool-current-health hand) 1000000000)\n")
    f.write("(= (tool-current-health wood-axe) 1000000000)\n")
    f.write("(= (tool-current-health bow) 1000000000)\n")

    k = 1
    for i in range(n):
        for j in range(n):
            f.write("(= (location-id m" + str(i) + "_" + str(j) + ") " + str(k) + ")\n")
            k = k + 1

    for x in range(1,6):
        i, j = randomxy()
        f.write("(thing-at-map tree  m" + str(i) + "_" + str(j) + ")\n")


    if skeleton_chance():
        sx, sy = randomxy()
        f.write("(thing-at skeleton)\n")
        f.write("(thing-at-loc skeleton  m" + str(sx) + "_" + str(sy) + ")\n")
        SKELETON_LOC = sx, sy

    if trap_chance():
        sx, sy = randomxy()
        f.write("(thing-at arrowtrap)\n")
        f.write("(thing-at-loc arrowtrap  m" + str(sx) + "_" + str(sy) + ")\n")
        TRAP_LOC = sx, sy

    # if monster_chance():
    #     sx, sy = randomxy()
    #     f.write("(thing-at monster)\n")
    #     f.write("(thing-at-loc monster m" + str(sx) + "_" + str(sy) + ")\n")
    #     MONSTER_LOC = sx, sy

    f.write("(thing-at-map shelter m2_0)\n")

    f.write(")")

    f.write("(:goal\n")
    f.write("(and\n")
    f.write("(= (thing-available wood) 5)\n")
    f.write(")))")

    return



def skeleton_chance():
    if random.random() <= SKELETON_CHANCE:
        return 1
    return 0


def trap_chance():
    if random.random() <= ARROW_TRAP_CHANCE:
        return 1
    return 0


def monster_chance():
    if random.random() <= MONSTER_CHANCE:
        return 1
    return 0


def randomxy():
    i = random.randint(0,n-1)
    j = random.randint(0,n-1)
    return (i, j)


if __name__ == "__main__":
    thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    MIDCA_ROOT = thisDir + "/../"

    ### Domain Specific Variables for JSHOP planner
    ff_DOMAIN_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/domain.pddl"
    ff_STATE_FILE = MIDCA_ROOT + "domains/ffdomain/minecraft/wood.1.pddl"

    # for i in range(1, 10):
    generate_file(ff_STATE_FILE)

