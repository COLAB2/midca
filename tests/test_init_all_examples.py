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
i = 0
for script_file in glob.glob(EXAMPLES_DIRECTORY+"*.py"):
    if not '__init__.py' in script_file:    
        i += 1
        #print "|=|=|=|=|=|=| "+str(script_file)+" |=|=|=|=|=|=|"
        script = subprocess.Popen(['python ',script_file],
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
        print "\n"+underline+"\n"+script_name+underline+"\n", err_output
        