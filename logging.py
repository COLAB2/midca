from __future__ import print_function
from datetime import datetime
import os, sys, copy

class Logger:
	
	logFolderOptions = ["log", "_log"]
	
	def __init__(self, keys = [], filesStayOpen = False):
		'''
		creates a new logger for a MIDCA run. The folder where the individual log files will be stored will be named based on the current date/time. It will be placed in ./log/, which will be created if it does not exist.
		
		Keys are both the filenames of actual log files and keys that will be passed to the logger to tell it where to log things. If no keys are passed in the default key will be "log"
		'''
		self.events = []
		self.defaultKey = "log"
		self.working = False
		self.filesStayOpen = filesStayOpen
		self.startTime = datetime.now()
		self.timeStr = str(self.startTime)
		if self.startTime.microsecond > 0:
			self.timeStr = self.timeStr[:-7]
		
		#create log dir if it does not exist. If there is a file at ./log, try ./_log. If neither works, fail and print an error message.
		folderFound = False
		self.cwd = os.getcwd()
		for logFolder in self.logFolderOptions:
			if folderFound:
				break
			self.logDir = os.path.join(self.cwd, logFolder)
			try:
				if os.path.isdir(self.logDir):
					folderFound = True
				else:
					if os.path.exists(self.logDir):
						print("Logger: file exists at " + self.logDir + ". Trying next option.", file = sys.stderr)
					else:
						os.mkdir(self.logDir)
						folderFound = True
			except OSError as e:
				print("Logger: error creating/accessing log directory: " + str(e), file = sys.stderr)
		if not folderFound:
			print("Logger: unable to create or find a log directory at any of: " + str([os.path.join(self.cwd, option) for option in self.logFolderOptions]) + ". Logging will be disabled.", file = sys.stderr)
		else:
			#now create the directory for this run
			self.thisRunDir = os.path.join(self.logDir, self.timeStr)
			try:
				os.mkdir(self.thisRunDir)
			except OSError as e:
				print("Logger: error creating log directory: " + str(e), file = sys.stderr)
			
			#now create the individual log file(s)
			self.files = {key: None for key in keys}
			self.working = True
			for key in keys:
				try:
					f = self.openFile(key)
					f.write("Log file for run starting at " + self.timeStr + "\n")
				except IOError as e:
					self.writeError(e, filename = os.path.join(self.thisRunDir, key), txt = "Log file for run starting at " + self.timeStr)
			if not self.files:
				self.working = False
			else:
				print("Logger: logging this run in " + self.thisRunDir, file = sys.stderr)
			if not self.filesStayOpen:
				for file in self.files.values():
					file.close()
	
	def openFile(self, key):
		f = open(os.path.join(self.thisRunDir, key), 'a')
		self.files[key] = f
		return f
	
	def _user_log(self, txt, keys = []):
		event = UserLogEvent(txt, keys)
		self.logEvent(event)
	
	def logEvent(self, event):
		event.time = datetime.now()
		if event.loggable:
			if not hasattr(event, 'keys') or not event.keys:
				keys = [self.defaultKey]
			elif event.keys == 'all':
				if self.files:
					keys = self.files.keys()
				else:
					keys = [self.defaultKey]
			else:
				keys = event.keys
			deltaTStr = str(event.time - self.startTime)
			deltaTStr = deltaTStr.lstrip(":0") + " - "
			for key in keys:
				self._write(deltaTStr + str(event), key)
		self.events.append(event)
	
	def log(self, val, key = None):
		if isinstance(val, basestring):
			self._user_log(val, key)
		elif isinstance(val, Event):
			self.logEvent(val)
		else:
			raise ValueError("log must get a string or an Event object")
	
	def _write(self, txt, key):
		if key not in self.files or not self.files[key] or self.files[key].closed:
			try:
				self.openFile(key)
			except IOError as e:
				self.writeError(e, filename = os.path.join(self.thisRunDir, key), txt = txt)
				return
		f = self.files[key]
		if f:
			f.write(txt + "\n")
			if not self.filesStayOpen:
				f.close()
	
	def writeError(self, e, filename = "", txt = ""):
		print("Logger: trying to write " + txt + " to file " + filename + "; got error " + str(e), file = sys.stderr)
	
	def close(self):
		for f in self.files.values():
			f.close()
	
	def logOutput(self):
		StdoutDirector(self)

class StdoutDirector:
    
    def __init__(self, logger):
        self.logger = logger
        self.stdout = sys.stdout
        sys.stdout = self
        self.current = ""
    
    #removes some color codes that mess up logging output
    def fixForMidca(self, s):
    	return s.replace("[94m", "").replace("[0m ", "")
    
    def write(self,s):
        self.current += self.fixForMidca(s)
        if self.current.endswith("\n"):
        	if len(self.current) > 1:
        		event = MidcaOutputEvent(self.current, ["log", "MIDCA output"])
        		self.logger.log(event)
        	self.current = ""
        self.stdout.write(s)
    
    def flush(self):
        self.stdout.flush()

class Event:
	
	def __init__(self, loggable = True, keys = []):
		self.loggable = loggable
		self.keys = keys
	
	def __str__(self):
		raise NotImplementedError("Event subclasses must implement __str__ or set loggable to False")

class MidcaOutputEvent(Event):
	
	def __init__(self, txt, keys = []):
		self.txt = txt
		self.loggable = True
		self.keys = keys
	
	def __str__(self):
		return self.txt

class UserLogEvent(Event):
	
	def __init__(self, txt, keys = []):
		self.txt = txt
		self.loggable = True
		self.keys = keys
	
	def __str__(self):
		return self.txt

class CycleStartEvent(Event):
	
	def __init__(self, cycle):
		self.cycle = cycle
		self.keys = 'all'
		self.loggable = True
	
	def __str__(self):
		s = "Starting cycle " + str(self.cycle) + "\n"
		return s
	
class CycleEndEvent(Event):
	
	def __init__(self, cycle):
		self.cycle = cycle
		self.keys = []
		self.loggable = False

class PhaseStartEvent(Event):
	
	def __init__(self, phase):
		self.phase = phase
		self.keys = "all"
		self.loggable = True
	
	def __str__(self):
		return "****** Starting " + str(self.phase) + " Phase ******\n"
	
class PhaseEndEvent(Event):
	
	def __init__(self, phase):
		self.phase = phase
		self.keys = []
		self.loggable = False
	
class ModuleStartEvent(Event):
	
	def __init__(self, module):
		self.module = module
		self.keys = ['log']
		self.loggable = True
	
	def __str__(self):
		s = "Running module " + str(self.module) + "\n"
		if "instance" in s:
			return s[:s.rindex("instance") - 1]
		else:
			return s
	
class ModuleEndEvent(Event):
	
	def __init__(self, module):
		self.module = module
		self.keys = []
		self.loggable = False

def test():
	l = Logger(["f1", "f2"])
	l._write("hello!", "f1")
