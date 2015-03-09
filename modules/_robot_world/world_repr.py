from abc import ABCMeta

class WorldState:
	pass
	

class Belief:
	
	def __init__(self, content, confidence, sources, timeFormed):
		self.content = content
		self.confidence = confidence
		self.sources = sources
		self.timeFormed = timeFormed
	
	def get_sources():
		return sources

class Location:
	
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

class LocationDistribution:
	
	__metaclass__ = ABCMeta
	
	
	@abstractmethod
	def add_observation(DetectionEvent):
		pass
	
	@abstractmethod
	def best_guess():
		'''
		Return a belief whose content is the most likely location
		in this distribution
		'''
		pass
	
	@abstractmethod
	def get_sources():
		'''
		Return the observations used to create this distribution.
		'''
		pass
	

class DetectedObject:
	pass

class DetectionEvent:
	pass

class DetectorProfile:

	__metaclass__ = ABCMeta
	
	@abstractmethod
	def false_positive_chance(detectionEvent):
		'''
		chance this observation is a false positive
		This is meant to reflect the chance that no object is in the given
		location, not the chance of a miscategorization
		'''
		pass
		
	
	@abstractmethod
	def miss_chance(detectionEvent):
		'''
		chance this observation would be missed in similar conditions
		'''
		pass
	
	
	@abstractmethod
	def miscategorization_chance(detectionEvent):
		'''
		chance that the type of the object detected is incorrect. If no type
		is given, this should be ignored.
		'''
		pass
	
	
	
	
LocationDistribution.register(tuple)
DetectorProfile.register(tuple)
	