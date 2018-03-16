# This file contains helpful functions for the nbeacons domain

import os
from MIDCA.modules._plan import pyhop
import re

'''
 
 
'''


def outermost_parentheses(input):
    return re.search("\((.*)\)", input).group(1)


def parenthetic_contents(string):
    """Generate parenthesized contents in string as pairs (level, contents)."""
    stack = []
    for i, c in enumerate(string):
        if c == '(':
            stack.append(i)
        elif c == ')' and stack:
            start = stack.pop()
            yield (len(stack), string[start + 1: i])


def PDDL_to_MIDCA_DOMAIN(pddl_file="domain.pddl", midca_file="a.txt"):
    file = open(pddl_file, "r")
    pddl = file.read()

    elements = list(parenthetic_contents(pddl))
    f = open(midca_file, 'w')

    for (d, elm) in elements:
        if (elm.startswith(":types")):
            lines = elm.split("\n")
            for line in lines[1:]:
                if line.strip() != "":
                    f.write("type(" + line.split("-")[0].strip() + ")\n")

        if (elm.startswith(":predicates")):

            lines = parenthetic_contents(elm)
            for (l, line) in lines:
                objs = ""
                types = ""
                pairs = line.split("?")
                f.write("\n predicate(" + pairs[0].strip() + ",")
                for pair in pairs[1:]:
                    obj_type = pair.split("-")
                    objs = objs + "," + obj_type[0].strip()
                    types = types + "," + obj_type[1].strip()

                f.write("[" + objs[1:].strip() + "], [" + types[1:].strip() + "])")

        if (elm.startswith(":action")):
            f.write("\n operator (")
            lines = elm.split(":")
            action_name = lines[1].split(" ")[1]
            f.write(action_name + ",")
            for line in lines[2:]:
                if line.startswith("parameters"):
                    parameters = outermost_parentheses(line).split("?")[1:]
                    print
                    parameters
                    p = ""
                    for parameter in parameters:
                        obj_type = parameter.split("-")
                        p = p + ", (" + obj_type[0].strip() + "," + obj_type[1].strip() + " )"

                    p = p[1:]
                    f.write("args = [" + p.strip() + "],")
                    print
                    "args = [" + p.strip() + "],"

                if line.startswith("precondition"):
                    f.write("preconditions = [")
                    precondition = list(parenthetic_contents(line))
                    for (l, pre) in precondition:
                        precon = ""
                        if l == 1:
                            # (player-at ?target)
                            condition = pre.split(" ")
                            f.write("condition(" + condition[0].strip())
                            for c in condition[1:]:
                                precon = precon + ", " + c.strip()

                            f.write(" [" + precon[1:].strip() + "]),\n")
                    f.write("],")
                if line.startswith("effect"):
                    f.write("results=[")
                    effect = list(parenthetic_contents(line))
                    for (l, pre) in effect:
                        precon = ""
                        if l == 1:
                            # (player-at ?target)
                            condition = pre.split(" ")
                            f.write("condition(" + condition[0].strip() + ",")
                            for c in condition[1:]:
                                precon = precon + ", " + c.strip()

                            f.write(" [" + precon[1:].strip() + "]),\n")
                    f.write("])")


if __name__ == "__main__":
    PDDL_to_MIDCA_DOMAIN()
#     precondition = list (parenthetic_contents(line))
#     for (l,p) in precondition:
#         print (l,p)
