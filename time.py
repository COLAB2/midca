from datetime import datetime
import multiprocessing

def now():
	return float((datetime.now() - epoch()).total_seconds())

def epoch():
	return datetime.utcfromtimestamp(0)

def run_for(timeout, func, *args, **kwargs):
	'''
	runs a function for timeout seconds or until it terminates. If not completed, the
	process will be terminated after timeout.
	'''
	process = multiprocessing.Process(target = func, args = args, kwargs = kwargs)
	process.start()
	process.join(timeout)
	if process.is_alive():
		process.terminate()
		process.join()
