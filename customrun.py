""" File: customrun.py
    Author: Dustin Dannenhauer (dustin.td@gmail.com)

    Create custom scenarios for MIDCA using xml files.The xml file
    describes a change to occur in the environment at a specific
    cycle. This could be removing or adding a predicate or executing a
    function that is defined in the class that using customrun.

    This module was created with the intention that custom run files
    would be used during the simulate module, but could "possibly" be
    used in any module.

    The order of changes (referred to as events in the xml file) is
    important and events are executed in that order.

    An example xml file is shown below:

    <?xml version="1.0"?>
    <events>
      <event tick="5">
	<change>onfire(B_)</change>
        <change>!onfire(C_)</change>
      </event>
      <event tick="10">
	<change>!FIRE-EXTINGUISHER(Fire Extinguisher 1)</change>
        <function signature="remove_fire_extinguisher">
	  <arg type="str">Fire Extinguisher 1</arg>
	</function>
        <function signature="start_random_fire"></function> 
      </event>
    </events>

    This example says that at cycle (tick) 5, first the predicate
    onfire(B_) should be added to the current state. Then onfire(C_)
    should be removed from the state. Then at cycle 10, first remove
    predicate FIRE-EXTINGUISHER(Fire Extinguisher 1) from the state,
    then execute remove_fire_extinguisher('Fire Extinguisher 1'), then
    execute start_random_fire().

    Any function that is referred to in the xml file, like
    start_random_fire() must be defined in the object (i.e. the
    simulator class) because that function will be called within that
    class like: 

    self.start_random_fire().

"""

""" Data structure of tick events is a dictionary where key is an
    integer of the cycle and value is a list of strings or lists,
    where the inner lists are composed of functions followed by their
    args, or if a string, the change to be made to the world state.

    For example:
    {5:[[start_random_fire,D_],onfire(B_),[start_random_fire[B_,C_]]}

    This means that at tick 5, first a start_random_fire(D_) will be
    called and then onfire(B_) will added into the world state, and
    then start_random_fire(B_,C_) will be called

"""

def load_custom_run_xml(calling_instance, file_name, verbose = 2):
        # the following code block checks that only valid class
        # functions can be called (small security check)
        valid_methods = []
        for data in inspect.getmembers(calling_instance, predicate=inspect.ismethod):
                #print "data is " + str(data)
                if not data[0].startswith('__'): # ignore class only methods
                        valid_methods.append(data[0])
        
        # a tick file is stored as a dictionary where keys are
        # integers representing current 'round' of the
        # simulation and the values are an array of functions
        # to be executed, in the order they are given from
        # left to right. These functions must be defined in
        # the arsonist class that will use the tick file.
        tick_events = {}
        
        # this is where we parse the xml
        tree = ET.parse(file_name)
        root = tree.getroot()
        assert root.tag == 'events'
        
        for child_event in root:
                assert child_event.tag == 'event'
                tick = int(child_event.attrib['tick'])
                funcs_this_tick = []
                print "---------------------- Tick " + str(tick) + " ----------------------------"
                for child_func_call in child_event:
                        curr_func_with_args = []
                        change_str = ""
                        print "child_func_call is " + str(child_func_call)
                        if child_func_call.tag == 'function':
                                signature = child_func_call.attrib['signature']
                                print "signature is " + str(signature)
                                print "valid methods are " + str(valid_methods)
                                assert signature in valid_methods
                                curr_func_with_args.append(signature)
                                print "Current children of child_func_call are "+str(list(child_func_call))
                                for child_arg in child_func_call.iter('arg'):
                                        arg_type = child_arg.attrib['type']
                                        arg = child_arg.text
                                        print "arg is " + str(arg) + " and arg_type is " + str(arg_type)
                                        print "eval string is: "+ arg_type+"(\""+arg+"\")"
                                        correctly_typed_arg = eval(arg_type+"(\""+arg+"\")")

                                        # need extra quotes around string data types
                                        if arg_type == 'str':
                                                correctly_typed_arg = "\"" + correctly_typed_arg + "\""
                                        curr_func_with_args.append(correctly_typed_arg)
                                funcs_this_tick.append(curr_func_with_args)
                        elif child_func_call.tag == 'change':
                                change_str = child_func_call.text
                                print "change_str is "+change_str
                                funcs_this_tick.append(change_str)
                tick_events[tick] = funcs_this_tick
                print "Just added for tick : "+ str(tick) + str(funcs_this_tick)
                        
        return tick_events

def load_custom_run_file(tickfile):
    tick_events = None
    if tickFile:
        if tickFile.endswith(".csv"):
            tick_events = load_custom_run_csv(self,tickFile)
        elif tickFile.endswith(".xml"):
            tick_events = load_custom_run_xml(self,tickFile)
        else:
            print "Custom run event file name:", tickFile, " does not appear to be a csv or xml file, please use an appropriate file ending."

    return tick_events

def run_events_on_cycle(cycle):
    # execute events from the tick file
    if cycle in self.tick_events.keys():
        for curr_event in self.tick_events[cycle]:
            print "curr_tick_events for cycle "+str(cycle)+ " is "+ str(curr_event)
            
            if isinstance(curr_event, str): # we know its a manual change
                print "we know curr_event is a string"
                self.apply_change(curr_event)
                
            elif isinstance(curr_event,list):
                print "we know curr_event is a list"
                func_name = curr_event[0]
                func_str = "self."
                func_str += func_name + "("
                args = curr_event[1:]
                for arg in args:
                    func_str += str(arg) + ","
                if len(args) > 0:
                    # remove the trailing comma (only if args is > 0)
                    func_str = func_str[0:len(func_str)-1] 
                func_str += ")"
                
                print "About to eval: ", func_str
                eval(func_str)
                # end tick file modifications


# deprecated
def load_tick_file_csv(calling_instance, file_name, verbose = 2):
        """Reads in a tick file from a csv and assumes all functions take no
           arguments (if you want to use arguments to functions, use
           the alternative function: load_tick_file_xml)

        """
        # this is a safety check to ensure that every call to eval
        # only happens on a function that is part of the simulator
        # class
        valid_methods = []
        for data in inspect.getmembers(calling_instance, predicate=inspect.ismethod):
                #print "data is " + str(data)
                if not data[0].startswith('__'): # ignore class only methods
                        valid_methods.append(data[0])
        
        # a tick file is stored as a dictionary where keys are
        # integers representing current 'round' of the
        # simulation and the values are an array of functions
        # to be executed, in the order they are given from
        # left to right. These functions must be defined in
        # the arsonist class that will use the tick file.
        tick_events = {}
        with open(file_name) as f:
                lines = f.readlines()
                for line in lines:
                        tokens = line.strip().split(',')
                        curr_tick = int(tokens[0])
                        curr_funcs = []
                        print "tokens[1:] is ", str(tokens[1:])
                        for f in tokens[1:]:
                                curr_funcs.append([f])
                        tick_events[curr_tick] = curr_funcs
                        if verbose >= 2: print "processed line: " + line
                        print "self.tick_events: " + str(tick_events)

        return tick_events
