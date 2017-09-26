#!/usr/bin/python


from Interval import Interval
import random


#
# Simple code for testing the Interval module, and showing how it is used
#

interval = Interval(1.5, 2.5)

for i in range(1, 100):
    interval.add(random.random() * 10)

print interval.getCount()

interval.clear()

print interval.getCount()

