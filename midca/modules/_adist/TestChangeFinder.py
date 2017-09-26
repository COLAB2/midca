#!/usr/bin/python


import random
from WindowPair import WindowPair
from Interval import Interval
from ADistance import ADistance
from ChangeFinder import ChangeFinder


#
# Simple code for testing the ChangeFinder module, and showing how it is used
#


# Add intervals to monitor to A-distance object
ad = ADistance()
data = []
for i in range(100):
    data.append(random.uniform(0.0, 1.0));
ad.addProportional(data, 0.1, 0.5)
print ad

# ad.add(Interval(-2, -1))
# ad.add(Interval(-1, 0))
# ad.add(Interval(0, 1))
# ad.add(Interval(1, 2))


# Create a change finder and add a single window pair
cf = ChangeFinder(ad)
cf.addWindowPair(WindowPair(100, 100, 0.5))


# Initialize the distance object
ad.init(cf)


cf.computeAlphas(ad.uniformSample(10000), 0.05, 50)


# Process 1000 samples 
i = 0;
for item in ad.uniformSample(1000):
    cf.addData(item)
    i = i + 1
    if cf.detectChange():
        print 'Change detected at sample ' + str(i)

# Process another 1000 samples from a slightly different distribution
for item in ad.uniformSample(1000):
    cf.addData(item + 1.0)
    i = i + 1
    if cf.detectChange():
        print 'Change detected at sample ' + str(i)
