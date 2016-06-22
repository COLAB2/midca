from __future__ import print_function
import copy, datetime, sys
import time
from MIDCA.mem import Memory
from MIDCA import goals, logging, trace
from MIDCA.worldsim import stateread
import threading

MAX_MODULES_PER_PHASE = 100

class Phase:

    def __init__(self, name):
        self.name = name

    def __str__(self):
        return self.name

    def __eq__(self, other):
        return type(self) == type(other) and self.name == other.name

    def __hash__(self):
        return hash(self.name)

class BaseModule:

    def init(self, mem, world = None):
        self.mem = mem

    def run(self, cycle, verbose = 2):
        '''
        This method will be called once per cycle and defines the
        behavior of a module.

        Note: The return value of the run method is only
        used in two special cases when MIDCA is run in interactive mode
        (see PhaseManager.run()). If the last module in a phase returns
        'continue', MIDCA will run the subsequent phase immediately. If
        it returns 'q', MIDCA will quit immediately. These special cases
        exist to allow modules a greater degree of control over their
        behavior in interactive mode.
        '''
        raise NotImplementedError("A MIDCA module must implement the \
        run(cycle) method.")

    def log(self, msg):
        if self.mem.logger:
            self.mem.logger.log(msg)
        else:
            print("Trying call BaseModule.log(), but Logging is not \
            enabled! To enable call midca.mem.enableLogging(logger)")

class MIDCA:

    def __init__(self, world = None, logenabled = True, logOutput = True,
                     logMemory = True, metaEnabled = False, phaseManager = None, verbose = 2):
        self.world = world
        self.mem = Memory()
        self.phases = []
        self.metaPhases = []
        self.modules = {}
        self.metaModules = {}
        self.verbose = verbose
        self.initialized = False
        self.phaseNum = 1
        self.metaPhaseNum = 1
        self.logger = logging.Logger(verbose=verbose)
        self.metaEnabled = metaEnabled
        self.mem.enableTrace()
        if metaEnabled:
            if not phaseManager:
                raise Exception("MetaEnabled but phaseManager pointer not given")
            self.mem.enableMeta(trace.CogTrace(), phaseManager)
        if not logenabled:
            self.logger.working = False
        else:
            self.logger.start()
            if self.logger.working:
                if logOutput:
                    self.logger.logOutput()
                self.mem.enableLogging(self.logger)
                self.mem.logEachAccess = logMemory

    def phase_by_name(self, name, meta = False):
        phases = self.phases
        if meta:
            phases = self.metaPhases

        for phase in phases:
            if phase.name == name:
                return phase
        return None

    def insert_phase(self, phase, phaseOrIndex, meta = False):
        phases = self.phases
        modules = self.modules
        if meta: # switch if inserting a meta phase
            phases = self.metaPhases
            modules = self.metaModules

        if isinstance(phase, str):
            phase = Phase(phase)
        if not isinstance(phase, Phase):
            raise KeyError(str(phase) + " is not a valid phase or phase name.")
        if isinstance(phaseOrIndex, str):
            phaseOrIndex = self.phase_by_name(phaseOrIndex, meta)
        elif isinstance(phaseOrIndex, int):
            phases.insert(phaseOrIndex, phase)
            modules[phase] = []
            return
        if not isinstance(phaseOrIndex, Phase):
            raise KeyError(str(phase) + " is not a valid phase or index.")
        if phaseOrIndex not in self.phases:
            raise KeyError("phase " + str(phaseOrIndex) + " not in phase list.")
        phases.insert(self.phases.index(phaseOrIndex), phase)
        modules[phase] = []

    def append_phase(self, phase, meta=False):
        if meta:
            self.insert_phase(phase, len(self.metaPhases) + 1, meta)
        else:
            self.insert_phase(phase, len(self.phases) + 1, meta)

    def remove_phase(self, phaseOrName):
        if isinstance(phaseOrName, str):
            phase = self.phase_by_name(phaseOrName)
        else:
            phase = phaseOrName
        try:
            self.phases.remove(phase)
            del self.modules[phase]
        except ValueError:
            raise ValueError("Phase " + str(phaseOrName) + " is not a phase.")
        #if there is a KeyError, something has gone very wrong.

    def append_module(self, phase, module, meta=False):
        self.insert_module(phase, module, MAX_MODULES_PER_PHASE, meta)

    def runtime_append_module(self, phase, module):
        self.runtime_insert_module(phase, module, MAX_MODULES_PER_PHASE)

    #note: error handling should be cleaned up - if a phase cannot be found by name, the error will report the phase name as "None" instead of whatever was given. True for removeModule as well.
    def insert_module(self, phase, module, i, meta=False):
        phases = self.phases
        modules = self.modules
        if meta:
            phases = self.metaPhases
            modules = self.metaModules
        if isinstance(phase, str):
            phase = self.phase_by_name(phase, meta)
        if phase not in phases:
            raise KeyError("phase " + str(phase) + " not in phase list. Call insert_phase() or append_phase() to add it.")
        if not hasattr(module, "run"):
            raise AttributeError("All modules must a 'run' function")
        if len(modules[phase]) == MAX_MODULES_PER_PHASE:
            raise Exception("max module per phase [" + str(MAX_MODULES_PER_PHASE) + "] exceeded for phase" + str(phase) + ". Cannot add another.")
        modules[phase].insert(i, module)

    # just like insert_module but also calls module.init()
    def runtime_insert_module(self, phase, module, i):
        if isinstance(phase, str):
            phase = self.phase_by_name(phase)
        if phase not in self.phases:
            raise KeyError("phase " + str(phase) + " not in phase list. Call insert_phase() or append_phase() to add it.")
        if not hasattr(module, "run"):
            raise AttributeError("All modules must a 'run' function")
        if len(self.modules[phase]) == MAX_MODULES_PER_PHASE:
            raise Exception("max module per phase [" + str(MAX_MODULES_PER_PHASE) + "] exceeded for phase" + str(phase) + ". Cannot add another.")
        self.modules[phase].insert(i, module)
        module.init(mem=self.mem, world=self.world, verbose=self.verbose)

    def removeModule(self, phase, i):
        if isinstance(phase, str):
            phase = self.phase_by_name(phase)
        if phase not in self.modules:
            raise KeyError("phase " + str(phase) + " not in phase list. Call insert_phase() or append_phase() to add it.")
        modules = self.modules[phase]
        if i < 0 or i >= len(modules):
            raise IndexError("index " + str(i) + " is outside the range of the module list for phase " + str(phase))
        else:
            modules.pop(i)

    def clearPhase(self, phaseOrName):
        if isinstance(phaseOrName, str):
            phase = self.phase_by_name(phaseOrName)
        else:
            phase = phaseOrName
        try:
            self.modules[phase] = []
        except ValueError:
            raise ValueError("Phase " + str(phaseOrName) + " is not a phase.")

    def get_modules(self, phase):
        if isinstance(phase, str):
            phase = self.phase_by_name(phase)
        if phase in self.modules:
            return self.modules[phase]
        else:
            raise ValueError("No such phase as " + str(phase))

    def init(self):
        self.init_cognitive_layer(verbose=self.verbose)
        if self.metaEnabled:
            self.init_metacognitive_layer(verbose=self.verbose)

    def init_cognitive_layer(self, verbose = 2):
        for phase in self.phases:
            modules = self.modules[phase]
            i = 0
            for module in modules:
                i += 1
                try:
                    if verbose >= 2:
                        print("[cognitive] Initializing " + phase.name + " module " + str(i) + " "+str(module.__class__.__name__)+ "...",end="")
                    module.init(world = self.world,
                                mem = self.mem)
                    if verbose >= 2: print("done.")

                except Exception as e:
                    print(e)
                    if verbose >= 2:
                        print("\n[cognitive] Phase " + phase.name + " module " + str(i) + " "+str(module.__class__.__name__)+ " has no init function or had an error. Skipping init.")

        self.initGoalGraph(overwrite = False)
        self.initialized = True

    def init_metacognitive_layer(self, verbose = 2):
        for phase in self.metaPhases:
            modules = self.metaModules[phase]
            i = 0
            for module in modules:
                i += 1
                try:
                    if verbose >= 2:
                        print("[metacognitive] Initializing " + phase.name + " module " + str(i) + " "+str(module.__class__.__name__)+"...",end="")
                    module.init(world = self.world,
                                mem = self.mem)
                    if verbose >= 2: print("done.")

                except Exception as e:
                    print(e)
                    if verbose >= 2:
                        print("\n[metacognitive] Phase " + phase.name + " module " + str(i) + " "+str(module.__class__.__name__)+ "has no init function or had an error. Skipping init.")

        self.initGoalGraph(overwrite = False)
        self.initialized = True

    def initGoalGraph(self, cmpFunc = None, overwrite = True):
        if overwrite or not self.mem.get(self.mem.GOAL_GRAPH):
            self.mem.set(self.mem.GOAL_GRAPH, goals.GoalGraph(cmpFunc))
            if self.verbose > 0: print("Goal Graph initialized.",)
            if cmpFunc:
                if self.verbose > 0: print()
            else:
                if self.verbose > 0:
                    print("To use goal ordering, call initGoalGraph manually with a custom goal comparator")

    def next_phase(self, verbose = 2, meta = False):
        phaseNum = self.phaseNum
        phases = self.phases
        modules = self.modules
        if meta: # switch if meta
            phaseNum = self.metaPhaseNum
            phases = self.metaPhases
            modules = self.metaModules

        retVal = ""
        self.phasei = (phaseNum - 1) % len(phases)
        if self.phasei == 0:
            self.logger.logEvent(logging.CycleStartEvent((phaseNum - 1) / len(phases)))
        if verbose >= 2:
            if meta:
                print("    ***[meta] Starting ", phases[self.phasei].name, "Phase ***\n", file = sys.stderr)
            else:
                print("****** Starting", phases[self.phasei].name, "Phase ******\n", file = sys.stderr)
            self.logger.logEvent(logging.PhaseStartEvent(phases[self.phasei].name))
        i = 0
        while i < len(modules[phases[self.phasei]]):
            module = modules[phases[self.phasei]][i]
            self.logger.logEvent(logging.ModuleStartEvent(module))
            try:
                retVal = module.run((phaseNum - 1) / len(phases), verbose)
                i += 1
            except NotImplementedError:
                if verbose >= 1:
                    print("module", module, "does not",
                          "implement the run() method and",
                          "is therefore invalid. It will be",
                          "removed from MIDCA.")
                self.removeModule(phases[self.phasei], i)
            self.logger.logEvent(logging.ModuleEndEvent(module))

        self.logger.logEvent(logging.PhaseEndEvent(phases[self.phasei].name))

        if not meta:
            self.phaseNum += 1
        else:
            self.metaPhaseNum += 1

        if (phaseNum - 1) % len(phases) == 0:
            self.logger.logEvent(logging.CycleEndEvent((phaseNum - 1) / len(phases)))

        # record phase and run metareasoner
        #self.mem.set("phase", self.phases[self.phasei].name)
        #metareasoner.MetaReasoner(self.trace, self.mem).run()

        return retVal

    def copy(self):
        '''
        This method does not make a true copy - it will not copy
        the original object's loggers and is not
        intended to be run, only checked to see what MIDCA's state was
        at an earlier time.
        '''
        newCopy = MIDCA(self.world, False, self.verbose)
        newCopy.mem = Memory()
        newCopy.mem.knowledge = self.mem.knowledge.copy()
        newCopy.mem.locks = {name: threading.Lock() for name in self.mem.locks}
        newCopy.phases = list(self.phases)
        newCopy.modules = self.modules.copy()
        newCopy.initialized = self.initialized
        newCopy.phaseNum = self.phaseNum
        return newCopy

class PhaseManager:

    def __init__(self, world = None, verbose = 2, display = None, storeHistory = False, metaEnabled = False):
        # phasemanager is passed in as a self pointer for metacognitive modification
        self.midca = MIDCA(world = world, verbose = verbose, metaEnabled = metaEnabled, phaseManager=self,logenabled=False)
        self.mem = self.midca.mem
        self.storeHistory = storeHistory
        self.history = []
        self.display = display
        self.twoSevenWarning = False
        self.logger = self.midca.logger
        self.meta_verbose = verbose

    '''
    convenience functions which wrap MIDCA functions
    '''
    def phase_by_name(self, name):
        return self.midca.phase_by_name(name)

    def insert_phase(self, phase, phaseOrIndex):
        self.midca.insert_phase(phase, phaseOrIndex)

    def append_phase(self, phase):
        self.midca.append_phase(phase)

    def append_meta_phase(self, phase):
        self.midca.append_phase(phase, meta=True)

    def get_phases(self):
        return [phase.name for phase in self.midca.phases]

    def append_module(self, phase, module):
        self.midca.append_module(phase, module)

    def append_meta_module(self, phase, module):
        self.midca.append_module(phase, module, meta=True)

    def runtime_append_module(self, phase, module):
        self.midca.runtime_append_module(phase, module)

    def insert_module(self, phase, module, i):
        self.midca.insert_module(phase, module, i)

    def insert_meta_module(self, phase, module, i):
        self.midca.insert_module(phase, module, i, meta=True)

    def remove_module(self, phase, i):
        self.midca.removeModule(phase, i)

    def clear_phase(self, phase):
        self.midca.clearPhase(phase)

    def get_modules(self, phase):
        return self.midca.get_modules(phase)

    def init(self):
        self.midca.init()

    def initGoalGraph(self, cmpFunc = None):
        self.midca.initGoalGraph(cmpFunc)
    '''
    functions for advancing through phases and complete cycles.
    '''

    def next_phase(self, verbose = 2):
        if self.storeHistory:
            self.history.append(self.midca.copy())
        val = self.midca.next_phase(verbose)
        return val

    def next_meta_phase(self, verbose = 2):
        # TODO - determine how to store history here
        #if self.storeHistory:
        #    self.history.append(self.midca.copy())
        if self.storeHistory and verbose >= 3: print("Warning: History not being stored during meta phase") #TODO
        val = self.midca.next_phase(self.meta_verbose, meta=True)
        return val


    def one_cycle(self, verbose = 1, pause = 0.5, meta=False, noInterface=True):
        phases = self.midca.phases
        if meta:
            phases = self.midca.metaPhases
        for i in range(len(phases)):
            t1 = datetime.datetime.today()
            if meta:
                self.next_meta_phase(verbose)
            else:
                self.next_phase(verbose)
            t2 = datetime.datetime.today()
            try:
                if (t2 - t1).total_seconds() < pause:
                    time.sleep(pause - (t2 - t1).total_seconds())
            except AttributeError:
                if not self.twoSevenWarning:
                    print('\033[93m' + "Use python 2.7 or higher to get accurate pauses between steps. Continuing with approximate pauses." + '\033[0m')
                    self.twoSevenWarning = True
                time.sleep(pause)

    def several_cycles(self, num, verbose = 1, pause = 0.01, meta=False, noInterface=True):
        for i in range(num):
            self.one_cycle(verbose, pause, meta, noInterface)

    def several_cycles_no_interface(self, num, verbose=1, pause = 0.01, meta=False):
        for i in range(num):
            self.one_cycle_no_interface(verbose, pause, meta)

    #MIDCA will call this function after the first phase. The function should take one input, which will be whatever is stored in self.midca.world.
    def set_display_function(self, function):
        self.display = function

    def clearWorldState(self):
        self.midca.world.objects = {}
        self.midca.world.atoms = []

    def applyStateChange(self, stateStr):
        stateread.apply_state_str(self.midca.world, stateStr)

    #function which runs MIDCA with a text UI
    def run(self,usingInterface=True):
        if not self.midca.initialized:
            raise Exception("MIDCA has not been initialized! Please call Midca.init() before running.")
        print("\nMIDCA is starting. Please enter commands, or '?' + enter for help. Pressing enter with no input will advance the simulation by one phase.")
        while 1:
            if usingInterface:
                print("Next MIDCA command:  ", file = sys.stderr, end = "")
                val = raw_input()
                print
                if val == "q":
                    break
                elif val == "skip":
                    self.one_cycle(verbose = 0, pause = 0)
                    print("cycle finished")
                elif val.startswith("skip"):
                                    #disable output and run multiple cycles
                    try:
                        num = int(val[4:].strip())
                        for i in range(num):
                            self.one_cycle(verbose = 0, pause = 0) # TODO - use several_cycles() instead?
                            #print("  Score is "+str(self.mem.get("Score")))
                            #print("  cycle "+str(i))
                            #self.display(self.midca.world)
                        print(str(num) + " cycles finished.")
                    except ValueError:
                        print("Usage: 'skip n', where n is an integer")
                elif val == "show":
                    if self.display:
                        try:
                            self.display(self.midca.world)
                        except Exception as e:
                            print("Error displaying world")
                    else:
                        print("No display function set. See PhaseManager.set_display_function()"    )
                elif val == "log":
                    print("Input the text to add to MIDCA's log file. Leave empty and press enter to cancel\n", file = sys.stderr)
                    txt = raw_input()
                    if txt:
                        self.logger.log(txt)
                elif val == "toggle meta verbose":
                    if self.meta_verbose > 0:
                        self.meta_verbose = 0
                        print("Turning OFF metacognitive phase outputs")
                    else:
                        self.meta_verbose = 2
                        print("Turning ON metacognitive phase outputs")
                elif val == "drawgoalgraph":
                    print("Input file name ending in .pdf or press enter to use default filename: goalgraph.pdf")
                    txt = raw_input()
                    if txt:
                        self.mem.get(self.mem.GOAL_GRAPH).writeToPDF(txt)
                    else:
                        self.mem.get(self.mem.GOAL_GRAPH).writeToPDF()
                elif val == "printtrace":
                    self.mem.trace.printtrace()
                elif val == "drawtrace":
                    print("Input file name ending in .pdf or press enter to use default filename: trace.pdf")
                    txt = raw_input()
                    if txt:
                        self.mem.trace.writeToPDF(txt)
                    else:
                        self.mem.trace.writeToPDF()
                elif val == "memorydump":
                    print("Please enter the variable you wish to see the values of, or hit enter to see all of them ")
                    txt = raw_input()
                    if txt:
                        keyfound = False
                        for key in self.mem.knowledge.keys():
                            if str(key) == txt:
                                keyfound = True
                                print("    ["+key+"] = "+str(self.mem.get(key))+"\n")
                        if not keyfound:
                            print("  Error: Key "+txt+" not found in MIDCA's memory")
                            print("  [Available Keys] "+str(self.mem.knowledge.keys()))
                    else:
                        print("  Current memory is: \n")
                        for key in self.mem.knowledge.keys():
                            print("    ["+key+"] = "+str(self.mem.get(key)))
                        print("")
                elif val == "worldstate":
                    print(str(self.mem.get("__world states")[-1]))
                    print("----- now printing self.midca.world -----")
                    print(str(self.midca.world))
                    print('----- and now: self.midca.world.atoms ----')
                    for atom in self.midca.world.atoms:
                        print("  "+str(atom))
                    print('----- and now: self.mem.get("__world states")[-1].atoms ----')
                    for atom in self.mem.get("__world states")[-1].atoms:
                        print("  "+str(atom))
                        
                elif val == "change":
                    print("Enter 'clear' to clear the world state, 'file' to input a state file name, or nothing to finish. Otherwise, enter changes to the world state. Use ! to negate atoms or remove objects, e.g. !on(A,B). Note that syntax is shared with state files in midca/worldsim/states, and each command must be on it's own line.")
                    while True:
                        input = raw_input("Next change:  ")
                        if not input:
                            break
                        elif input == "clear":
                            self.clearWorldState()
                            print("World state cleared")
                        elif input == "file":
                            print("Enter the name of a valid state file, or leave blank to cancel.")
                            filename = raw_input()
                            if filename == "":
                                print("File load cancelled")
                                continue
                            s = ""
                            try:
                                s = open(filename).read()
                            except IOError:
                                print("Cannot open file")
                            try:
                                self.applyStateChange(s)
                                print("State loaded")
                            except Exception as e:
                                print("Error loading state. State may be partially loaded: ", str(e))
                        else:
                            try:
                                self.applyStateChange(input)
                                print("Change applied")
                            except Exception as e:
                                print(e)
                elif val == "?" or val == "help":
                    print("interface: \n enter/return -> input commands. Empty command goes to next cycle \n q -> quit \n skip n -> skips n cycles \n show -> print world representation \n change -> modify or clear world state \n log -> log some text \n ? or help -> show this list of commands \n")
                elif val:
                    print("command not understood")
                else:
                    val = self.next_phase()
                    if self.mem.metaEnabled:
                        metaval = self.one_cycle(verbose = 2, pause=0.01, meta=True)
                    if val == "continue":
                        self.next_phase()
                    elif val == "q":
                        break
                    if self.mem.metaEnabled:
                        if metaval == "continue":
                            self.next_meta_phase() # TODO - not sure when this gets called
            else: # not using interface - TODO # duplicate code, clean this code up
                val = self.next_phase()
                if self.mem.metaEnabled:
                    metaval = self.one_cycle(verbose = 2, pause=0.01, meta=True)
                if val == "continue":
                    self.next_phase()
                elif val == "q":
                    break
                if self.mem.metaEnabled:
                    if metaval == "continue":
                        self.next_meta_phase() # TODO - not sure when this gets called


        print("MIDCA is quitting.")
