MIDCA1.02: NOT intended to be merged with MIDCA1.01.

Documentation to follow, for now see demo_instructions.txt for instruction on running MIDCA demo.

I) Memory
	
	-Memory is implemented as a map between keys and values. Keys may not be collections but otherwise can be basically any value (strings are used in default implementation); values can be arbitrary structures of any kind. Note that these are the same rules used by python dicts.
	
	-Within each module, self.mem should refer to the shared MIDCA memory passed into the init() function at startup.
		-this assignment should occur in the init() function called when MIDCA starts.
	
	-Within each module, self.memKeys should refer to the set of values declared in custom/memconstants.py and passed into the init() function at startup.

	1) Modifying memory
		
		-To STORE a value in memory, call self.mem.set(key, value)
		
		-To APPEND a value to a list stored in memory, call self.mem.add(listKey, newValue). 
			-This will create a list containing the item if there was no previous value stored under listkey
			-if a non-list oldValue was stored under listkey, this will create a two-item list [oldValue, newValue]
		
		-to REMOVE a value from memory, call self.mem.remove(key)
		-to CLEAR memory, call self.mem.clear(). Use this sparingly.
	
	2) Accessing memory
		
		-to retrieve a value (or data structure) from memory, call self.mem.get(key). This will return None if there is no value stored.
	
	3) memKeys
	
		-for convenience, keys that will be used by multiple modules can be declared in the MIDCA_ROOT/custom/memconstants.py file. These values can then be accessed from self.memKeys within a module.
		
		Usage example: 
			1) Add to memconstants.py the line: 
				MY_KEY = 'key1'
			2) From any module, call 
				self.mem.set(self.memKeys.MY_KEY, val) 
				or
				self.mem.get(self.memKeys.MY_KEY)

Memory example:

	Goal: want to generate a score in the Eval module, then retrieve it, and all previous scores calculated, in the Intend module. The score will be computed using values stored by other code under the keys defined by self.memKeys.K1 and self.memKeys.K2.

	Step 1: add score memory key to custom/memconstants.py. Simply add the line:
		MY_SCORE_KEY = 'myscore'

	Step 2: retrieve input values from memory. In the run() method of the eval module, add the lines
		input1 = self.mem.get(self.memKeys.K1)
		input2 = self.mem.get(self.memKeys.K2)
	
	Step 3: calculate score using input values, and store result. This assumes a function scoreFunc(input1, input2) which returns a score. Underneath the above lines, add the line:
		self.mem.add(self.memKeys.MY_SCORE_KEY, scoreFunc(input1, input2))
		#note that if we wanted to only keep the most recent score, we would use self.mem.set(...)

	Step 4: retrieve score values. In the Intend module's run() function, add the line:
		scores = self.mem.get(self.memKeys.MY_SCORE_KEY)

	To get the last score calculated, you could use:
		lastScore = scores[-1] #scores is a python list

	Final note: if an attempt is made to retrieve a value before one is stored, it will return None, and this should be handled.


II) Custom modules

	1) Any python object can be a module so long as it defines the methods init(self, world, mem, memKeys) and run(verbose).
	
	2) If the module needs to interact with MIDCA's memory, its init() function should store the values of mem and memKeys, e.g. include the lines:
		self.mem = mem
		self.memKeys = memKeys
	
	3) To add a module M to MIDCA, where M is an instance of a class meeting the requirements above and instantiates a phase named 'P':
		
		-Open the file MIDCA_ROOT/custom/customsetup.py
			-add 'P' to the PHASES list (near the top of the file) at the point in the cycle it is to be executed.
			-in function get_modules(options), add the line:
				modules['P'] = M
		-note that the module will ONLY be loaded if both of these steps are done AND if the names match exactly.
