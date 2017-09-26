
This folder should contain all modules to be used in MIDCA. This is not necessary, but is recommended for ease of understanding. 

** Important note on module implementation:

A MIDCA module is not the same as a python module. A module in MIDCA is an object which implements a phase of the MIDCA cycle. Any class used as a module must have methods "init(world, mem)" and "run(verbose)." Note that "init" != "__init__". Also note that mem should be saved in the init method, or else the module will have no access to MIDCA's memory.
