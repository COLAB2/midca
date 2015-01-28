# This file is a draft of an API between the Meta-cognitive and
# Cognitive Layers in MIDCA 

# Author: Dustin Dannenhauer

# There seems to be 2 categories of actions: Structural and Knowledge-based:
# Structural would be to re-arrange modules
# Knowledge based would be to change goals of obj level


############################
## Perception

def get_current_state_of_obj_level():
    """ Returns an overview of the obj level, including:
        - Available modules
        - Current module in each phase
        - Memory (including goals, etc)
    """

def get_internal_trace_by_module(m):
    """ Returns the internal trace of module m """

def get_external_trace_by_module(m):
    """ Returns the external trace of module m """

def get_memory():
    """ Returns the complete memory that entire object level uses """

def get_goals():
    """ Returns the current goals object level is using """

def get_memory_by_module(m):
    """ Returns the memory used by module m """

############################
## Structural Action

def get_available_modules():
    """ Returns all available modules """

def get_active_modules():
    """ Returns the current module used in each phase """

def get_inactive_modules():
    """ These are all the available modules that are not active for a given phase """

def set_module_for_phase(m, p):
    """ Replaces the current module m in phase p, and the old module is kept available but not active """

def add_module_for_phase(m,p):
    """ Adds a module that will now be available for phase p, but will not be active """

def rem_module_for_phase(m,p):
    """ Removes module m from phase p, regardless if it is active/inactive """

def set_modules_by_order(p,mlist): #mlist = [m1,m2,...,m_n] where m_i is a module
    """ Sets the current phase to use the modules in mlist in that order """

############################
## Knowledge Action

def set_goals(glist):
    """ Set the goals of the obj level to be the given goals """

def add_goal(g):
    """ Add goal g to obj level goals """

def rem_goal(g):
    """ Remove goal g from obj level goals"""




