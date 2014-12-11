predicateworld.py: defines a set of MIDCA instantiations - using the PhaseManager class - for the built-in predicate logic world simulator and planner (pyhop). These objects can then be run either programmatically using the API or in interactive mode.

simple_run: Runs a simple version of MIDCA with only user-defined goal generation

simple_run_arson: Adds to simple_run TF-Tree goal generation and an arsonist who sets fires

run_extinguish: adds to simple_run the need to be holding a fire extinguisher to put out a fire

cogsci_demo: the example from a 2014 MIDCA paper. Incorporates TF-Trees, a simulation of Meta-AQUA used to generate goals to apprehend the arsonist, and a scoring system for analyzing MIDCA's success in tower construction