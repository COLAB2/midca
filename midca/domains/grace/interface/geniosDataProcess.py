import pandas as pd
from loc import LatLong2LocalGrid

def write_file(filename, items):
    with open(filename, "w") as file:
        for item in items:
            file.write(item)

def split_files(path, filename, index):

    result = [[]]
    with open(path+filename) as bigfile:
        for lineno, line in enumerate(bigfile):
            if line == "\n" and not flag:
                flag = True
                result.append([])
            if not line == "\n":
                flag = False
                result[-1].append(line)
    for i in range(len(result)):
        write_file(path+index[i], result[i])




def process():
    pd.set_option('display.max_rows', 500)
    pd.set_option('display.max_columns', 500)
    pd.set_option('display.width', 1000)
    try:
        path = "genios_midca/"

        split_files(path, "genios_out_loc_latest.txt", index = ['loc.txt', 'actual_trajectory.txt'])
        # reading csv file
        loaded_values = pd.read_csv(path + "loc.txt",
                    delimiter=',',names = ['longitude', 'latitude', 'time_stamp'], skiprows=1,
                                    usecols=[0,1,2])
        #print(loaded_values)
        loaded_values = loaded_values.fillna(-999)

        lat = loaded_values['latitude']
        long = loaded_values['longitude']
        time_stamp = loaded_values['time_stamp']

        xylist = []


        #for i in range(len(lat)):
        #    xylist.append(LatLong2LocalGrid(lat[i],long[i]))

        return lat[0], long[0], time_stamp[0]
    except IOError:
        return -1,-1,-1

def searchprocess():
    pd.set_option('display.max_rows', 500)
    pd.set_option('display.max_columns', 500)
    pd.set_option('display.width', 1000)

    try:
        path = "genios_midca/"
        split_files(path, "genios_out_search_loc_latest.txt", index = ['loc.txt', 'search.txt', 'actual_trajectory.txt'])
        # reading csv file
        loaded_values = pd.read_csv(path + "search.txt",
                    delimiter=',',names = ['target_position', 'actual_time'], skiprows=1,
                                    usecols=[0,1])

        total_search_locations = len(loaded_values)
        loaded_values = loaded_values[loaded_values.actual_time != -1]
        completed_search_locations = len(loaded_values)

        if total_search_locations == completed_search_locations:
            return True
        else:
            return False
    except IOError:
        return False

if __name__ == '__main__':
    #path = "genios_midca/"
    #filename = path + "genios_out_loc_latest.txt"
    #split_files(path, filename)
    print(searchprocess())
