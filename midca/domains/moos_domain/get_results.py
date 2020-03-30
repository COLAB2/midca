from __future__ import division
from os import listdir
from os.path import isfile, join


mypath = "results_random/"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]


scores = {}
for file in onlyfiles:
    f = open(mypath + file, "r")
    for line in f.readlines():
        line = line.replace("\n", "")
        value = line.split(",")
        value  = [int(each) for each in value]
        if value[0] in scores:
            scores[value[0]].append(value[1])
        else:
            scores[value[0]] = [value[1]]

# get average

for key, value in scores.items():
    print value
    scores[key] = (float(sum(value)) / float(len(value))) * 10

# get x axis
x_axis = []
y_axis = []
for key in sorted(scores.keys()):
    x_axis.append(key)
    y_axis.append(scores[key])

print ("X-AXIS : {}".format(x_axis))
print ("Y-axis : {}".format(y_axis))


