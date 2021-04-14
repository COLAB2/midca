predicateworld.py: defines a set of MIDCA instantiations - using the
PhaseManager class - for the built-in predicate logic world simulator
and planner (pyhop). These objects can then be run either
programmatically using the API or in interactive mode.

-----------------

simple_run: Simulation of tower construction in blocksworld. Runs a
simple version of MIDCA with only user-defined goal generation.

run_extinguish: Simulation of tower construction in blocksworld. Adds
to simple_run the need to be holding a fire extinguisher to put out a
fire.

simple_run_arson: Simulation of tower construction and arson
prevention in blocksworld. Adds to simple_run TF-Tree goal generation
and an arsonist who sets fires.

cogsci_demo: The example from a 2014 MIDCA paper. Incorporates
TF-Trees, a simulation of Meta-AQUA used to generate goals to
apprehend the arsonist, and a scoring system for analyzing MIDCA's
success in tower construction.

cogsci_demo_ma: Like cogsci_demo but connects to Meta-AQUA instead of
using simulated goal generation.

chicken_run: Simple example domain where chickens can cross a
road. Goals must be input by the user using the text interface in the
interpret phase. Try onleft(clucky).

restaurant_demo: Simulation of restaurant activities including taking
orders, preparing dishes, and serving the food. An example goal is
order_serve(A_,HOT_DOG). This demo will automatically generate goals
for MIDCA and then Intend selects the best subset of the goals given a
budget of $50 for the food. 

nbeacons_demo: Simulation of the NBEACONS domain (adapted from
marsworld in [Dannenhauer and Munoz-Avila 2015]). Some examples of
goals to give during the interpret phase when prompted for a goal are
agent-at(Curiosity, Tx3y7) and agent-at(Curiosity, Tx15y2).
