import pandas as pd
from loc import LatLong2LocalGrid
def process():
    pd.set_option('display.max_rows', 500)
    pd.set_option('display.max_columns', 500)
    pd.set_option('display.width', 1000)

    path = "genios_midca/"
    # reading csv file
    loaded_values = pd.read_csv(path + "sim_traj1.txt",
                delimiter=',',names = ['longitude', 'latitude', 'time_stamp'], skiprows=1,
                                usecols=[0,1,2])
    #print(loaded_values)

    lat = loaded_values['latitude']
    long = loaded_values['longitude']
    time_stamp = loaded_values['time_stamp']

    xylist = []


    for i in range(len(lat)):
        xylist.append(LatLong2LocalGrid(lat[i],long[i]))

    return xylist

#print(process())
