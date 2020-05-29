'''
This is the start of the script for users to easily create domains from their .PDDL files
Based on the file PDDL_util.py from Zoreh
'''

import sys
import inspect
import shutil  # copy file
import os
import re

# Setup for directory navigation
newDomainName = ""
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
MIDCA_ROOT = thisDir + "/../"
newDirectoryRoot = ""


# Parsing function for PDDL to .sim
def outermost_parentheses(input):
    return re.search("\((.*)\)", input).group(1)


# Parsing function for PDDL to .sim
def parenthetic_contents(string):
    """Generate parenthesized contents in string as pairs (level, contents)."""
    stack = []
    for i, c in enumerate(string):
        if c == '(':
            stack.append(i)
        elif c == ')' and stack:
            start = stack.pop()
            yield (len(stack), string[start + 1: i])


# Parsing function for PDDL to .sim
def PDDL_to_MIDCA_DOMAIN(pddl_file, midca_file):
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
                    # print(parameters)
                    p = ""
                    for parameter in parameters:
                        obj_type = parameter.split("-")
                        p = p + ", (" + obj_type[0].strip() + "," + obj_type[1].strip() + " )"

                    p = p[1:]
                    f.write("args = [" + p.strip() + "],")
                    # print("args = [" + p.strip() + "],")

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


# Function for creating __init__.py files at the specified directory
def create_init(location):
    # create an empty __init__.py file at the specified directory
    f = open(location + "/__init__.py", "w")


# Function for creating the runnable file in the examples folder and update the readme
def update_examples_directory():
    # uses the example_template.txt
    shutil.copy(thisDir + "/Domain_Parsing/templates/examples_template.txt",
                MIDCA_ROOT + "/examples/" + newDomainName + "_run.py")

    # TODO: regex replace "<domain-name>" within the template with the new domain name
    # print("New Domain Name: " + newDomainName)


# Function regex replaces "<domain-name>" within the file specified with the new domain name
def regex_filename(file):
    pass


if __name__ == "__main__":
    # get <domain-name>/.pddl file to convert and pull the domain name from
    PDDL_file = sys.argv[1]
    newDomainName = re.split('/', PDDL_file)[0]  # TODO: get domain name with either / or \ file scheme, only does /

    newDirectoryRoot = thisDir + "/" + newDomainName

    if not os.path.exists(newDirectoryRoot + "/problems"):
        os.makedirs(newDirectoryRoot + "/problems")  # create the new problems sub-directory

    create_init(newDirectoryRoot)  # create the empty __init__.py file needed in the new domain directory
    update_examples_directory()

    # TODO: pass the .ppdl filename to the function to open the correct one
    # TODO: reopen the below function call
    PDDL_to_MIDCA_DOMAIN(PDDL_file, newDirectoryRoot + "/" + newDomainName + ".sim")
