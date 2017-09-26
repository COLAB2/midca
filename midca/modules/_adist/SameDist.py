#!/usr/bin/python



import random
import math



#
# Return the value of the cumulative density function for a standard normal
# at location x.  I got this code from here:
#
#   http://www.johndcook.com/python_phi.html
#
def gaussianCDFStandard(x):
    a1 =  0.254829592
    a2 = -0.284496736
    a3 =  1.421413741
    a4 = -1.453152027
    a5 =  1.061405429
    p  =  0.3275911

    sign = 1
    if x < 0:
        sign = -1

    x = abs(x)/math.sqrt(2.0)

    t = 1.0/(1.0 + p*x)
    y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*math.exp(-x*x)

    return 0.5*(1.0 + sign*y)



#
# Return the value of the cumulative density function for a normal with 
# mean m and standard deviation s at location x.
#
def gaussianCDF(x, m, s):
    return gaussianCDFStandard((x - m) / s)



#
# Return the probability of making an error in rejecting the nulll hypothesis
# (Type I error) that two normals have the same mean.  The normals are
# summarized by their respective means, m1 and m2, and standard deviations,
# s1 and s2.  This assumes a two-tailed test.  That is, if the null
# hypothesis is wrong, m1 is equally likely to be greater than or less than m2.
#
def sameMean(m1, s1, m2, s2):
    m = m1 - m2
    v = s1 * s1 + s2 * s2

    p = gaussianCDF(0, m, v**0.5)
    p = min(p, 1 - p)
    return 2 * p
    


#
# Return the probability of making an error in rejecting the nulll hypothesis
# (Type I error) that two normals have the same variance.  The inputs are the
# sample sizes, n1 and n2, and standard deviations, s1 and s2, of the two
# distributions.  This is an implementation of Bartlett's test, which is
# described here:
#
#   http://en.wikipedia.org/wiki/Bartlett's_test
#
def sameVariance(n1, s1, n2, s2):

    n = n1 + n2    # Total number of samples
    v1 = s1 * s1   # Variance of first sample
    v2 = s2 * s2   # Variance of second sample

    # Pooled variance
    v = ((n1 - 1) * v1 + (n2 - 1) * v2) / (n - 2)

    # Compute chi-squared value per Bartlett
    x2 = (n - 2) * math.log(v) - (n1 - 1) * math.log(v1) - (n2 - 1) * math.log(v2)
    x2 = x2 / (1 + ((1 / (n1 - 1) - 1 / (n - 2)) + ((1 / (n2 - 1) - 1 / (n - 2)))) / 3)

    # Because we have just two samples, we compare the square root of the 
    # chi-squared statistic to a standard normal.  For details, look here:
    # 
    #   http://onlinestatbook.com/chapter14/distribution.html
    #
    return 1 - gaussianCDFStandard(x2**0.5)



#
# Return the probability of making an error in rejecting the null hypothesis
# (Type I error) that two samples came from the same generating normal 
# distribution.  For each sample, the sample size, sample mean, and sample
# standard deviation must be specified.
#
def sameDist(n1, m1, s1, n2, m2, s2):
    p1 = sameMean(m1, s1, m2, s2)
    p2 = sameVariance(n1, s1, n2, s2)

    print str(p1) + ' ' + str(p2)

    # I make an error in rejecting the null hypothesis about the distribution
    # if I make a Type I error for the means (let that event be A) and a 
    # Type I error for the variances (let that event be B).  Then the
    # probability of a Type I error is p(A and B) which, because A and B are
    # independent due to Cochran's theorem, is p(A) * p(B).
    return p1 * p2


