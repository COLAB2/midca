import numpy as np
import zmq
import time


class Minelayer:

    def __init__(self):
        # for Zmq
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5999")

        # we dont want to have the mines with the same lables
        self.label_count = 0 # for the mine labels

    def send_message(self, taglist):
        time.sleep(0.1)
        for i, point in enumerate(taglist):
            content = [b"AddHazard", b"x=" + str(point[0]) + ",y=" + str(point[1]) + ",label= " + str(self.label_count) + ", type=benign"]
            self.publisher.send_multipart(content)
            self.label_count +=1

    def write_to_file(self, taglist, filename = "/home/sampath/moos-ivp/moos-ivp-midca/missions/multi_agent/tags.txt"):
        time.sleep(0.1)
        f = open(filename, "a")
        for i, point in enumerate(taglist):
            #content = [b"AddHazard", b"x=" + str(point[0]) + ",y=" + str(point[1]) + ",label= " + str(self.label_count) + ", type=benign"]
            #self.publisher.send_multipart(content)
            content = "hazard = x=" + str(point[0]) + ",y=" + str(point[1]) + ",label= " + str(self.label_count) + ", type=benign\n"
            f.write(content)
            self.label_count +=1
        f.close()
