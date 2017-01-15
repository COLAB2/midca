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
import sys

EXAMPLES_DIRECTORY = 'examples/'
NUM_PROCESSES = 1 # number of python processes to run in parallel
FILES_TO_IGNORE = ['__init__','predicateworld','homography','baxter'] # will ignore any file containing one of these

# WARNING: if the run delay is ever too short (i.e. it takes longer than the delay for midca to execute
# the skip command, this whole script will deadlock. Therefore the CUSTOM_RUN_DELAYS should be used for
# any script that needs more time

SKIP_COMMAND = 'skip 100'
DEFAULT_RUN_DELAY = 8
CUSTOM_RUN_DELAYS = {'nbeacons_aaai17_agent3':60}

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
        
    found_exception = False
            
    if not ignore_file:
        i += 1
        script_name = '{:.<60}'.format("examples/"+script_file)
        sys.stdout.write(script_name)
        #print "|=|=|=|=|=|=| "+str(script_file)+" |=|=|=|=|=|=|"
        script = subprocess.Popen(['python ', '-u', EXAMPLES_DIRECTORY+script_file],
                                  stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,
                                  bufsize=-1)
         
        time.sleep(2) # give MIDCA enough time to load all modules
        
        # Read ten lines of stderr, if any exceptions or errors, failed!
        NUM_STDERR_LINES_TO_READ = 10
        i = 0
        while script.poll():
            output_line = script.stderr.readline()
            if "Error" in output_line or "Traceback" in output_line or "Exception" in output_line:
                found_exception = True
                break
            if i >= NUM_STDERR_LINES_TO_READ:
                i+=1
                break
                
        #sys.stdout.write("finished checking errors")
            #print output_line 
        #while len(output_line) > 0:
        #    print output_line
#         print "post while loop"
#         all_output = script.stdout.read()
#         for line in all_output:
#             if 'Exception' in line or 'Traceback' in line:
#                 found_exception = True
#             print line
        #print "found_exception is "+str(found_exception)
        if found_exception:
            sys.stdout.write('{:.>60}'.format('[FAILED during initialization]\n'))
            script.kill()
        else:
            sys.stdout.write('{:.<10}'.format('init'))
            sys.stdout.write('{:.<5}'.format('run'))            
            #sys.stdout.write("........init....run")
            
            script.stdin.write('skip 100 \n')
            DELAY = DEFAULT_RUN_DELAY 
            for k,v in CUSTOM_RUN_DELAYS.items():
                if k in script_name:
                    DELAY = v
            for i in range(DELAY):
                time.sleep(1)
                #sys.stdout.write(".")
            
            i = 0
            while script.poll():
                output_line = script.stderr.readline()
                if "Error" in output_line or "Traceback" in output_line or "Exception" in output_line:
                    found_exception = True
                    break
                if i >= NUM_STDERR_LINES_TO_READ:
                    i+=1
                    break
                    
#             all_output = script.stderr.readlines()
#             for line in all_output:
#                 if 'Exception' in line or 'Traceback' in line:
#                     found_exception = True
#                 print line
            #stdout_value, stderr_value = script.communicate('skip 100')
            
            #err_output = repr(stderr_value).replace('\\r\\n','\n').replace('\'','').strip()
            #if len(err_output) > 0:
            #    pass
            #print ".........ran 100 cycles",
            if not found_exception:
                stdout_value, stderr_value = script.communicate('q') # to quit MIDCA
        

        #script_name = str(i)+". "+script_file+" Exceptions:\n"
        #underline = "-"*len(script_name)
        #err_output = repr(stderr_value).replace('\\r\\n','\n').replace('\'','').strip()
        
        #if err_output == 'Next MIDCA command:':
        #    err_output = ''
            if found_exception:
                sys.stdout.write('{:.>44}\n'.format('[FAILED while running]'))
                #sys.stdout.write("....[FAILED while running]\n")
            else:
                sys.stdout.write('{:.>44}\n'.format('[SUCCEEDED]'))
                #sys.stdout.write("....[SUCCEEDED]\n")
                
            