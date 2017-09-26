#!/usr/bin/python


from Window import Window 


#
# A pair of data windows across which changes are to be detected
#
# 06/05/09 JTO - Created
#
class WindowPair:


    #
    # Create a new WindowPair object
    #
    def __init__(self, n1, n2, alpha):
        self.w1 = Window(n1)     # The first window in the pair
        self.w2 = Window(n2)     # The second window in the pair
        self.alpha = alpha       # Threshold used for detecting differences
                                 # in data in windows


    #
    # Create a string representation of a WindowPair object
    #
    def __repr__(self):
        s = 'w1: ' + str(self.w1) + '\nw2: ' + str(self.w2) + '\n'
        s = s + 'alpha: ' + str(self.alpha) + '\n'
        return s


    __str__ = __repr__


    #
    # Remove all data from the pair of windows
    #
    def clear(self):
        self.w1.clear()
        self.w2.clear()


    # 
    # Add a data item to the window pair, adding data to the first window
    # until it is full and then adding data to the second window
    #
    #   item - The data item to add
    #
    def add(self, item):
        if not self.w1.isFull():
            self.w1.add(item)
        else:
            self.w1.add(None)
        self.w2.add(item)


    #
    # Return one of the windows in the pair
    #
    #   which - Setting this value to 0 causes the first window to be
    #           returned.  Any other value results in the second window
    #           being returned.
    #
    def getWindow(self, which):
        if which == 0:
            return self.w1
        else:
            return self.w2


    #
    # Return true iff both windows in the pair are full
    #
    def isFull(self):
        return self.w1.isFull() and self.w2.isFull()
