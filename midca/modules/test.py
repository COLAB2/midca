from __future__ import division
from midca import midcatime
from scipy.stats import poisson
import pandas as pd



def calculate_bins(repeat_times, interval=50, verbose=3):
    max_value = max(repeat_times)
    bins = [i for i in range(0,int(max_value+interval), interval)]
    if verbose > 2:
        print ("The Bin Intervals are", bins)
    return bins

def binning(repeat_times, bins, verbose=3):
    result =  pd.cut(repeat_times, bins, retbins=False)
    if verbose >2:
        print ("Result after binning is", result)
    return result

def CalculateLambda(bins, verbose=3):
    time_intervals = { }
    for each in bins:
        if each in time_intervals:
            time_intervals[each] +=1
        else:
            time_intervals[each] = 1
    # calculate lambda
    avg_rate = sum(time_intervals.values())/len(time_intervals)
    if verbose >2:
        print ("The Lambda for Poisson is", avg_rate)
    return avg_rate

def negative_slopes(poisson_rates, verbose=3):
    for i in range(1, len(poisson_rates)):
        if poisson_rates[i-1] > poisson_rates[i]:
            if verbose > 2:
                print ("The Negative Slope is ", i-1)
            return i-1

    return len(poisson_rates)



#print( b[0])


#print(b)

#print (poisson.pmf(6, 2.6))

rectime = midcatime.now()

ontology = {
                "block" : {
                "agent_threat": 0,
                "mission_threat": 0,
                "repeat": [],
                },

                "remora" : {
                "agent_threat": 1,
                "mission_threat": 0,
                "repeat": [],
                },
}

def calculate_anomaly_repeat(anomaly, threshold = 3):
        time = midcatime.now() - rectime
        ontology[anomaly]["repeat"].append(time)
        repeat_times = ontology[anomaly]["repeat"]

        print ("The Input is", repeat_times)
        if len(repeat_times) > 1:
            bin_interval = calculate_bins(repeat_times)
            bins = binning(repeat_times, bin_interval)
            rate = CalculateLambda(bins)

            # calculate poisson rates
            poisson_rates = [poisson.pmf(i, rate) for i in range(10)]

            # identify the negative slope
            repetition_peak = negative_slopes(poisson_rates)

            if repetition_peak >= threshold:
                return 1

        return 0

def main():
    while (True):
        print ("Chance of Occuring Anomaly",calculate_anomaly_repeat("block"))
        b = raw_input("")

main()
