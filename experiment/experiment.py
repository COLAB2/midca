'''
Created on Jul 16, 2015

@author: dustin
'''

import time
from multiprocessing import Pool

class Experiment():
    '''
    The Experiment object is used to set up and run experiments where
    MIDCA is configured and run many times, and data is collected for
    each run. This class is a way to make it easier to do run these
    kinds of experiments.

    Recommended directory for storing new data:
    ../MIDCA/experiments/<new-folder>

    Guidelines for setting up an experiment:
    '''

    def __init__(self, name):
        self.runs = []
        self.datawritefuncs = [] # TODO - change this to be a single element
        self.destructfuncs = []
        self.name = name
        self.run_count = 0

    def addDataFile(self, filename):
        '''
        Adds a data file for output to be written to. Currently only
        one data format is supported: CSV
        '''
        if filename and filename.endswith(".csv"):
            self.filenames.append(filename)
        else:
            raise Exception("Tried to use output data file that is NOT a .csv file: "+filename)


    def appendRun(self, midca, num_cycles):
        '''
        Appends a new MIDCA run. Every run will be executed in the order
        it was added using this function and the MIDCA object after running
        MIDCA will be passed to the data output function.
        '''
        # Note: these are the arguments passed to singlerun() defined inside run() below
        this_run_args = [self.run_count, midca, num_cycles, self.datawritefuncs[0]]
        self.runs.append(this_run_args)
        self.run_count+=1

    def addWriteDataFunc(self, func):
        '''
        Adds a function that will be called on the MIDCA object after it has
        been executed according to its parameters. The function must take
        a single argument - MIDCA. The ideal use is that the function object
        will read data values stored in MIDCA's memory that have been updated
        after MIDCA has run and then store those in a file.
        '''
        self.datawritefuncs.append(func)

    def addDestructFunc(self, func):
        '''
        Adds function to be run at the end, before the program exits (i.e.
        close file handles, move or store data files)
        '''
        self.destructfuncs.append(func)

    def setNumCycles(self, numCycles):
        '''
        Number of cycles for each MIDCA instance to execute
        '''
        self.numCycles = numCycles

    def run(self):
        '''
        Executes all the runs from appendRun and sends the resulting midca
        objects to each data writing function added by addWriteDataFunc()
        '''
        
        # func for Pool.map() (there are some restrictions on the kinds of functions it can take)
        def singlerun(args):
            run_id = args[0]
            midca_inst = args[1].createMIDCAObj()
            num_cycles = args[2]
            data_write_func = args[3]
            curr_midca = midca_inst.getMIDCAObj()
            curr_midca.init()
            midca_inst.run_cycles(num_cycles)
            result = data_write_func(run_id, num_cycles, curr_midca)
            print(result) 
            return result

        
        # Uses multiprocessing to give each run its own python process
        pool = Pool(processes=1)
        # NOTE: it is very important chunksize is 1 (each MIDCA must use its own python process)
        print("self.run[0]="+str(map(str,self.runs[0])))
        results = pool.map(singlerun, self.runs, chunksize=1)
        print("Experiment finished. We've obtained "+str(len(results))+" data points")
        return results
        
#         
#         print "Running experiment: " + self.name
#         for run_id in range(len(self.runs)):
#             print "Run #" + str(run_id)
#             # execute as a function because MIDCA
#             # is wrapped in an anonymous lambda to ensure it executes at runtime
#             midcaInst = self.runs[run_id]
#             curr_midca = midcaInst.getMIDCAObj()
#             curr_midca.init()
#             print(midcaInst)
#             midcaInst.run_cycles(self.numCycles)
#             print(str(self.numCycles) + " cycles finished.")
#             print(midcaInst)
#             for datawritefunc in self.datawritefuncs:
#                 datawritefunc(run_id, midcaInst.getMIDCAObj())
#                 print "Just wrote out data"
#                 #time.sleep(1)
#             print(midcaInst)
#             # delete to save space (so we are not holding on to midca's memory)
#             #del curr_midca # i believe this only deletes the copy of midca, the original is still in the list self.runs
# 
#         # run destruct functions
#         for dfunc in self.destructfuncs:
#             dfunc()


