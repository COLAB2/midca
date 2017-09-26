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
od.addAlpha(0.99)


# Create a change finder and add a single window pair
cf = ChangeFinder(od)
cf.addWindowPair(WindowPair(200, 200, 0.99))


f = open('md4', 'r')
i = 0
for item in f.readlines():
    cf.addData(float(item))
    i = i + 1
    if cf.detectChange():
        print 'Change detected at sample ' + str(i)

f.close()
