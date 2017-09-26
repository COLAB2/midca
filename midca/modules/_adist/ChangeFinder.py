


import random


#
# An object that implements the Kifer, Ben-David, and Gherke algorithm
# for finding changes in data streams.
#
# 06/07/09 JTO - Created
#
class ChangeFinder:


    #
    # Create a new ChangeFinder object
    #
    #   distanceObj - An object that supports the various methods required to
    #                 compute the distance between pairs of windows of data
    #
    def __init__(self, distanceObj):
        self.distanceObj = distanceObj
        self.windowPairs = []


    #
    # Create a string representation of a ChangeFinder object
    #
    def __repr__(self):
        s = ''
        for wp in self.windowPairs:
            s = s + str(wp) + '\n'
        return s


    __str__ = __repr__


    #
    # Add a window pair to be monitored 
    #
    #   wp - The window pair
    #
    def addWindowPair(self, wp):
        self.windowPairs.append(wp)


    #
    # Return the window pairs
    #
    def getWindowPairs(self):
        return self.windowPairs


    #
    # Process the next data item from the stream
    #
    #   item - The data item
    #
    def addData(self, item):
        for wp in self.windowPairs:
            wp.add(item)
        self.distanceObj.update(item)


    #
    # Reset the state of the ChangeFinder to that before it processed any data
    #
    def clear(self):
        self.distanceObj.clear()
        for wp in self.windowPairs:
            wp.clear()
        

    # 
    # Return a list of the current distances for all window pairs
    #
    def getDistances(self):
        distances = []
        
        for wp in self.windowPairs:
            
            if wp.isFull():
                distances.append(self.distanceObj.distance(wp))
            else:
                distances.append(0.0)

        return distances


    #
    # Return true if a distribution change is detected across any of the
    # window pairs
    #
    def detectChange(self):
        change = 0
        
        for wp in self.windowPairs:
            
            # Ensure that both windows are full before testing
            if wp.isFull():
                dist = self.distanceObj.distance(wp)
                if (dist > wp.alpha):
                    change = 1

        if change:
            self.clear()

        return change


    #
    # Use a randomization test to compute alpha values for the window pairs.
    # Given a sample of data, shuffle the data in place, process the stream
    # through a window pair, and remember the largest distance between the
    # two windows.  Repeat this a large number of times and sort the
    # maximum distances to find a given percentile cutoff.
    #
    #   data - The sample of data
    #   p - The percentile
    #   n - The number of times to iterate the process
    #
    def computeAlphas(self, data, p, n):

        self.clear()

        distances = {}
        maxDistances = {}

        for wp in self.windowPairs:
            maxDistances[id(wp)] = []

        print 'Computing alpha values: ' + str(len(data)) + ' data items, ' + \
              'p = ' + str(p) + ', n = ' + str(n)

        # Iterate the desired number of times
        for i in range(n):

            if (i % int(n / 10) == 0):
                print '% done: ' + str(i * 100 / n)

            for wp in self.windowPairs:
                distances[id(wp)] = []

            random.shuffle(data)

            for item in data:
                self.addData(item)
                for wp in self.windowPairs:
                    if wp.isFull():
                        distances[id(wp)].append(self.distanceObj.distance(wp))

            for wp in self.windowPairs:
                maxDistances[id(wp)].append(max(distances[id(wp)]))

        i = 1
        for wp in self.windowPairs:
            maxDistances[id(wp)].sort()
            wp.alpha = maxDistances[id(wp)][int(n * (1 - p))]
            print 'Window pair #' + str(i) + ': alpha = ' + str(wp.alpha)

        self.clear()
