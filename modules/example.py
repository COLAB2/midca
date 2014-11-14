
class Example:
	
	def init(self, world, mem):
		#save MIDCA memory object for reference. Most modules should do this.
		self.mem = mem
	
	def run(self, cycle, verbose):
		#load state from memory
		states = self.mem.get(self.mem.STATES)
		try:
			state = states[-1]
		except AttributeError, IndexError:
			#no state recorded
			state = None
		if verbose > 1:
			print "State at cycle", cycle, ":"
			print str(state)
		#save new knowledge to memory by key
		#key must not conflict with any key used in this or other module
		myValue = 5
		self.mem.set("myKey", myValue)
		
		#note no return value