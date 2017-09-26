#!/usr/bin/python


#
# Defines a closed interval on the real line
#
# 06/06/09 JTO - Created
#
class Interval:


    #
    # Create a new Interval object
    #
    #   low - Lower bound
    #   high - Upper bound
    #
    def __init__(self, low, high):
        self.low = low
        self.high = high
        self.count = 0


    #
    # Create a string representation of an Interval object
    #
    def __repr__(self):
        s = '[' + str(self.low) + ', ' + str(self.high) + ']'
        s = s + ' n = ' + str(self.count)
        return s


    __str__ = __repr__


    #
    # Determine if a data item falls inside the interval and, if so,
    # increment the interval's counter
    #
    #   item - The data item
    #
    def add(self, item):
        if item != None and item >= self.low and item <= self.high:
            self.count = self.count + 1


    #
    # Determine if a data item falls inside the interval and, if so,
    # decrement the interval's counter
    #
    #   item - The data item
    #
    def remove(self, item):
        if item != None and item >= self.low and item <= self.high:
            self.count = self.count - 1


    # 
    # Reset the count of the number of items that fell into the interval
    # to zero
    #
    def clear(self):
        self.count = 0


    # 
    # Return the current count of data items that fell inside the interval
    #
    def getCount(self):
        return self.count
