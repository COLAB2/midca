#!/usr/bin/python


from WindowPair import WindowPair


#
# Simple code for testing the WindowPair module, and showing how it is used
#

wp = WindowPair(5, 5, 0.5)

for item in range(20):
    wp.add(item)
    print(wp)
    
