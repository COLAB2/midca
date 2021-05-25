#!/usr/bin/python


#
# A window of data from a data stream
#
# 06/05/09 JTO - Created
#
class Window:


    #
    # Create a new Window object
    #
    #   n - The maximum number of data items the window can hold
    #
    def __init__(self, n):
        self.n = n              # Maximum number of items window can hold
        self.data = []          # The data stored in the window
        self.lastIn = None      # The last data item added to the window
        self.lastOut = None     # The last data item pushed out by a new item


    #
    # Create a string representation of a Window object
    #
    def __repr__(self):
        first = 1
        s = '['
        for item in self.data:
            if not first:
                s = s + ' '
            first = 0
            s = s + str(item)
        return s + ']'


    __str__ = __repr__


    #
    # Return true if the window is full, else return false
    #
    def isFull(self):
        return len(self.data) >= self.n


    #
    # Add a data item to a Window.  If the window is full, the oldest
    # item in the window is discarded.
    #
    #   item - The data item to add.  When the value is None, both the
    #          lastIn and lastOut fields of the window are set to None.
    #
    def add(self, item):

        if item == None:
            self.lastIn = None
            self.lastOut = None
        else:
            self.data.append(item)
            self.lastIn = item
            self.lastOut = None
            while len(self.data) > self.n:
                self.lastOut = self.data.pop(0)


    #
    # Remove all of the data items stored in a Window
    #
    def clear(self):
        self.data = []
        self.lastIn = None
        self.lastOut = None


    #
    # Return a specific data item stored in a Window
    #
    # i - The index of the desired item
    #
    def getItem(self, i):
        return self.data[i]


    #
    # Return the data stored in the window
    #
    def getData(self):
        return self.data


    #
    # Return the last data item pushed out of the window by a new item
    #
    def getLastOut(self):
        return self.lastOut


    #
    # Return the last data item added to the window
    #
    def getLastIn(self):
        return self.lastIn


    #
    # Return the maximum number of data items the window can hold
    #
    def getSize():
        return self.n
