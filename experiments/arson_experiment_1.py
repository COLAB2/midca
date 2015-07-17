'''
Created on Jul 16, 2015

@author: dustin
'''
from MIDCA.experiment import experiment

class ArsonCogSciDemo():
    '''
    This is a custom experiment where MIDCA is run by varying the following:
    - probability of fire
    - using TFTrees for putoutfire goals
    - using simulated metaaqua to generate goal to put out arsonist
    '''


    def __init__(self, params):
        '''
        Setup code for experiment
        '''
        ex = experiment.Experiment()
        ex.appendRun()

    def createMIDCAObject(self, arsonchance):