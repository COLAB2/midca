'''
Created on Jul 16, 2015

@author: dustin
'''

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

    def __init__(self):
        self.runs = []
        self.datawritefuncs = []

    def addDataFile(self, filename):
        '''
        Adds a data file for output to be written to. Currently only
        one data format is supported: CSV
        '''
        if filename and filename.endswith(".csv"):
            self.filenames.append(filename)
        else:
            raise Exception("Tried to use output data file that is NOT a .csv file: "+filename)


    def appendRun(self, midca):
        '''
        Appends a new MIDCA run. Every run will be executed in the order
        it was added using this function and the MIDCA object after running
        MIDCA will be passed to the data output function.
        '''
        self.runs.append(midca)

    def addWriteDataFunc(self, func):
        '''
        Adds a function that will be called on the MIDCA object after it has
        been executed according to its parameters. The function must take
        a single argument - MIDCA. The ideal use is that the function object
        will read data values stored in MIDCA's memory that have been updated
        after MIDCA has run and then store those in a file.
        '''
        self.datawritefuncs.append(func)


    def run(self):
        '''
        Executes all the runs from appendRun and sends the resulting midca
        objects to each data writing function added by addWriteDataFunc()
        '''
        for run_i in self.runs:
            pass

