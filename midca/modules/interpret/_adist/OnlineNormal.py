#!/usr/bin/python



import random



class OnlineNormal:


    def clear(self):
        self.normalizer = 0.0
        self.valsum = 0.0
        self.devsum = 0.0

        
    def __init__(self, alpha):
        self.alpha = alpha
        self.clear()


    def __repr__(self):
        s = '[m = ' + str(self.getMean()) + ', s = ' + str(self.getVariance() ** 0.5) + ', n = ' + str(self.getNormalizer()) + ']'
        return s


    __str__ = __repr__


    def update(self, x):
        self.normalizer = self.alpha * self.normalizer + 1
        self.valsum = self.alpha * self.valsum + x
        self.devsum = (self.valsum / self.normalizer - x) ** 2 + \
            self.alpha * self.devsum


    def getMean(self):
        return self.valsum / self.normalizer


    def getVariance(self):
        return self.devsum / self.normalizer


    def getNormalizer(self):
        return self.normalizer
