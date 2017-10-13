#!/usr/bin/python



from Window import Window
from WindowPair import WindowPair
from OnlineNormal import OnlineNormal
import SameDist



class OnlineDist:


    def __init__(self):
        self.alphas = []


    def __repr__(self):
        s = '[ '

        for alpha in self.alphas:
            s = s + str(alpha) + ' '
            
        s = s + ']'

        return s


    __str__ = __repr__


    def addAlpha(self, alpha):
        self.alphas.append(alpha)


    def update(self, x):
        pass


    def clear(self):
        pass


    def distance(self, wp):
        pMin = 1.0

        for alpha in self.alphas:
            leftNormal = OnlineNormal(alpha)
            rightNormal = OnlineNormal(alpha)

            for x in wp.w1.getData():
                leftNormal.update(x)
            for x in wp.w2.getData():
                rightNormal.update(x)

            print leftNormal
            print rightNormal
            print ' '

            p = SameDist.sameDist(leftNormal.getNormalizer(), leftNormal.getMean(), leftNormal.getVariance() ** 0.5, rightNormal.getNormalizer(), rightNormal.getMean(), rightNormal.getVariance() ** 0.5)

            if (p < pMin):
                pMin = p

        return 1 - pMin
