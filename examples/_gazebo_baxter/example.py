import os,inspect
thisDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/../"

print(thisDir)
