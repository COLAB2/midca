ACCESS = "Access" #when could read, write, or both
READ = "Read"
WRITE = "Write"

EVENT = "Event"
START = "Process Start"
END = "Process End"

class Event:
	
	def __init__(self, eventType = EVENT, **kwargs):
		self.eventType = eventType
		for key, val in kwargs.items():
			setattr(self, key, val)
		self.dir = list(kwargs.keys())
		self.dir.append('eventType')
	
	def __dir__(self):
		return self.dir
	
	def __str__(self):
		if hasattr(self, 'text') and self.text:
			return self.text
		return "Event: " + str({attr: getattr(self, attr) for attr in dir(self)})

class LogEvent(Event):
	
	def __init__(self, text = "", **kwargs):
		newKwargs = dict(kwargs)
		newKwargs['text'] = text
		Event.__init__(self, eventType = EVENT, **newKwargs)

class MemEvent(Event):
	
	def __init__(self, memKey, memAccessType = ACCESS, **kwargs):
		newKwargs = dict(kwargs)
		newKwargs['memKey'] = memKey
		newKwargs['memAccessType'] = memAccessType
		Event.__init__(self, eventType = EVENT, **newKwargs)
	
	def __str__(self):
		s = "Memory " + self.memAccessType + ", key = " + str(self.memKey)
		return s

class ProcessStart(Event):
	
	def __init__(self, process, **kwargs):
		newKwargs = dict(kwargs)
		newKwargs['process'] = process
		Event.__init__(self, eventType = START, **newKwargs)
	
	def __str__(self):
		return "Starting " + str(self.process)

class ProcessEnd(Event):
	
	def __init__(self, process, **kwargs):
		newKwargs = dict(kwargs)
		newKwargs['process'] = process
		Event.__init__(self, eventType = END, **newKwargs)
	
	def __str__(self):
		return "Ending " + str(self.process)
		
	
e = Event(memType = 1)
print dir(e)
print e

mem = MemEvent("Goals")
print dir(mem)
print mem

log = LogEvent("Logging")
print dir(log)
print log

start = ProcessStart("p1")
end = ProcessEnd("p1")
print start, end