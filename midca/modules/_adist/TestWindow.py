#!/usr/bin/python


from Window import Window


#
# Simple code for testing the Window module, and showing how it is used
#

# Create window of size 10
w = Window(10)

# Add integers 0 - 19 to the window
for i in range(20):
    w.add(i)

print w
print w.isFull()
print w.getItem(5)
print w.lastOut
w.clear()
print w
print w.isFull()
print w.lastOut
