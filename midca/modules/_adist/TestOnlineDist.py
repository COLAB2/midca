#!/usr/bin/python


from WindowPair import WindowPair
from Interval import Interval
from OnlineDist import OnlineDist
from ChangeFinder import ChangeFinder
import random


#
# Simple code for testing the OnlineDist module with the ChangeFinder
#


# Create the online dist object and add alphas for online normals
od = OnlineDist()
od.addAlpha(0.95)
od.addAlpha(0.99)


# Create a change finder and add a single window pair
cf = ChangeFinder(od)
cf.addWindowPair(WindowPair(200, 200, 0.99))


# Process 1000 samples 
i = 0;
for k in range(1000):
    item = random.gauss(10, 1)
    cf.addData(item)
    i = i + 1
    if cf.detectChange():
        print 'Change detected at sample ' + str(i)

# Process another 1000 samples from a slightly different distribution
for k in range(1000):
    item = random.gauss(13, 1)
    cf.addData(item)
    i = i + 1
    if cf.detectChange():
        print 'Change detected at sample ' + str(i)
