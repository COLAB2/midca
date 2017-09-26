#!/usr/bin/python


import sys
import random
import copy
from Interval import Interval
import ChangeFinder


#
# Instances of this class can be used to compute the A-distance (as defined
# by Kifer, Ben-David, and Gehrke) between two windows of univariate,
# real-valued data.  See the ChangeFinder class for more information.
#
# 06/06/09 JTO - Created
#
class ADistance:


    #
    # Create a new ADistance object
    #
    def __init__(self):
        self.intervals = []     # Intervals on the real line to monitor
        self.cf = None          # ChangeFinder object that uses this object
        self.intervalDict = {}  # Map from windows to lists of intervals
        self.relativized = True # Compute relativized discrepancy


    #
    # Create a string representation of an ADistance object
    #
    def __repr__(self):
        
        s = ''

        if (self.cf == None):
            s = 'Intervals:\n'
            for interval in self.intervals:
                s = s + str(interval) + '\n'
        
        else:
            i = 1
            for wp in self.cf.getWindowPairs():
                s = s + 'Window pair #' + str(i) + ':\n'
                s = s + 'w1:\n'
                for interval in self.intervalDict[id(wp.getWindow(0))]:
                    s = s + str(interval) + '\n'
                s = s + 'w2:\n'
                for interval in self.intervalDict[id(wp.getWindow(1))]:
                    s = s + str(interval) + '\n'

        return s


    __str__ = __repr__


    #
    # Add an interval to the ADistance object
    #
    #   interval - The interval to add
    #
    def add(self, interval):
        self.intervals.append(interval)


    #
    # Given a window of data and a proportion p in (0, 1), add intervals 
    # such that each interval spans the desired proportion of instances in 
    # the window of data.  The windows overlap by a proportion o in [0, 1).
    # For example, when p = 0.1 and o = 0.5, each interval will span 10% of 
    # the data in the window, the last 50% of the instances in window i
    # will be the first 50% of the instances in window i + 1.
    #
    def addProportional(self, data, p, o):
        data.sort()
        width = int(len(data) * p)
        offset = int(width * (1.0 - o))
        index = 0
        while (index + width < len(data)):
            low = data[index]
            if index + offset + width >= len(data):
                high = data[len(data) - 1]
            else:
                high = data[index + width]
            self.add(Interval(low, high))
            index = index + offset


    #
    # Initialize an ADistance object.  
    #
    #   cf - The ChangeFinder object with which it will work
    #
    def init(self, cf):

        self.cf = cf

        # Create a dictionary that pairs single windows with intervals
        for wp in self.cf.getWindowPairs():
            self.intervalDict[id(wp.getWindow(0))] = \
                copy.deepcopy(self.intervals);
            self.intervalDict[id(wp.getWindow(1))] = \
                copy.deepcopy(self.intervals);


    #
    # This method is called for the ADistance object every time a new data
    # item is added to the ChangeFinder with which this object works
    #
    #   item - The data item
    #
    def update(self, item):

        # Loop over all window pairs in the ChangeFinder
        for wp in self.cf.getWindowPairs():

            # Get intervals associated with first window in pair
            window = wp.getWindow(0)
            intervals = self.intervalDict[id(window)]

            # Update intervals
            for interval in intervals:
                interval.add(window.getLastIn())
                interval.remove(window.getLastOut())

            # Get intervals associated with first window in pair
            window = wp.getWindow(1)
            intervals = self.intervalDict[id(window)]

            # Update intervals
            for interval in intervals:
                interval.add(window.getLastIn())
                interval.remove(window.getLastOut())


    #
    # This method should be called for the ADistance object every time
    # the associated ChangeFinder is cleared
    #
    def clear(self):
        for intervals in self.intervalDict.itervalues():
            for interval in intervals:
                interval.clear()


    #
    # Compute the A-distance between two windows in a pair
    #
    #   wp - The window pair
    #
    def distance(self, wp):

        vals = []

        # Get the window objects
        window1 = wp.getWindow(0)
        window2 = wp.getWindow(1)

        # Get the intervals for each window
        intervals1 = self.intervalDict[id(window1)]
        intervals2 = self.intervalDict[id(window2)]

        # Compute the distance between each interval
        for interval1, interval2 in zip(intervals1, intervals2):

            p1 = float(interval1.getCount()) / float(window1.n)
            p2 = float(interval2.getCount()) / float(window2.n)

            if p1 - p2 == 0.0:  # Protects against numerical issues
                val = 0.0
            else:
                val = abs(p1 - p2)
                if self.relativized:
                    val = val / pow(min((p1 + p2) / 2, 1 - (p1 + p2) / 2), 0.5)
            
            vals.append(val)
            
        # Return largest distance
        return max(vals)


    #
    # Draw a uniform sample from the smallest closed intervals that span
    # all of the intervals in an ADistance object
    #
    #   n - The size of the sample
    #
    def uniformSample(self, n):
        
        sample = []
        low = sys.maxint
        high = -sys.maxint

        # Compute bounds on enclosing interval
        for interval in self.intervals:
            if interval.low < low:
                low = interval.low
            if interval.high > high:
                high = interval.high

        for i in range(n):
            sample.append(random.uniform(low, high))

        return sample
