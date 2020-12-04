import json
import os

def write_to_file(way_points):
        with open('communication/Goal_seq.txt', 'w') as outfile:
            json.dump(way_points, outfile)
        try:
            os.remove("communication/Waypoint_seq.txt")
        except:
            pass

def read_json(fname = "communication/Waypoint_seq.txt"):
    try:
        with open(fname) as f:
            data = json.load(f)
            return data["Waypoint_sequences"]
    except IOError:
        return None


if __name__ == "__main__":
    """"
    data = {}
    data["InitialCoordinates"] = [100, 200]
    data["DestCoordinates"] = [300, 400]
    write_to_file(data)
    """

    print (read_json())

