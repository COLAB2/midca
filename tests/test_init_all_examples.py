'''
Created on Oct 18, 2016

@author: Dustin

This is a script that will run any script found in examples/ .
Essentially, this tests whether or not initialization of MIDCA
succeeded. 
'''

import glob, os 
import subprocess
import time

EXAMPLES_DIRECTORY = 'examples/'
NUM_PROCESSES = 1 # number of python processes to run in parallel
FILES_TO_IGNORE = ['__init__','predicateworld',] # will ignore any file containing one of these

i = 0

os.chdir(EXAMPLES_DIRECTORY)
script_files = glob.glob("*.py")

# go back to top level MIDCA dir
os.chdir('../')

for script_file in script_files:
    # ignore certain files
    ignore_file = False
    for ign in FILES_TO_IGNORE:
        if ign in script_file:
            ignore_file = True
            
    if not ignore_file:
        i += 1
        #print "|=|=|=|=|=|=| "+str(script_file)+" |=|=|=|=|=|=|"
        script = subprocess.Popen(['python ',EXAMPLES_DIRECTORY+script_file],
                                  stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,)
        time.sleep(1) # give MIDCA enough time to load all modules
        stdout_value, stderr_value = script.communicate('q') # to quit MIDCA
        script_name = str(i)+". "+script_file+" Exceptions:\n"
        underline = "-"*len(script_name)
        err_output = repr(stderr_value).replace('\\r\\n','\n').replace('\'','').strip()
        
        if err_output == 'Next MIDCA command:':
            err_output = ''
        print "\n"+underline+"\n"+script_name+underline, err_output
        