from utils import blockstate, scene

class PerfectOberver:

	'''
	MIDCA Module which copies a complete world state. It is designed to interact with the built-in MIDCA world simulator. To extend this to work with other representations, modify the observe method so that it returns an object representing the current known world state.
	'''

	def init(self, world, mem, memKeys):
		self.world = world
		self.mem = mem
		self.memKeys = memKeys
	
	#perfect observation
	def observe(self):
		return self.world.copy()
		
	def run(self, verbose = 2):
		world = self.observe()
		if not world:
			raise Exception("World obervation failed.")
		self.mem.add(self.mem.STATES, world)
		repr = str(self.world_repr(world))
		if verbose >= 1:
			print "World observed."