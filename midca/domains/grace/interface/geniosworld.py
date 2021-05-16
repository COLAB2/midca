from __future__ import division
import time
from simulator import Simulator
import threading
import numpy as np
import loc
import json
import send_email, receive_email
from geniosDataProcess import process, searchprocess

np.random.seed(555)

class TagWorld():

    def __init__(self, name="genios" ):

        # initialize simulator
        self.name = name
        self.simulator = Simulator(start_x=0, start_y=0, side=1600, size_m=4, size_n = 3)

        #search complete
        self.searching = "false"
        self.speed = 0

        # lock for threading
        self.lock = threading.Lock()
        self.Recvlock = threading.Lock()

    def TagWorld(self):
        return self

    def startsim(self):
        # construct the message
        loc = self.get_cell()
        loc = loc.split(",")
        self.move_cell([0,0], [int(loc[0]) , int(loc[1])])

    def endSim(self):
        pass

    def move_cell(self, initial_position, final_destination):
        self.lock.acquire()

        #get the real moos coordinate equivalent of cell coordinates
        x,y = self.simulator.grid_to_sim(final_destination[0], final_destination[1])
        x,y = loc.LocalGrid2LatLong(x,y)
        data = {}
        data["Request"] = "move"
        data["DestinationCoordinates"] = [[x,y]]
        with open('data.json', 'w') as outfile:
                json.dump(data, outfile)

        send_email.send(data["Request"])
        self.lock.release()

    def search(self, position, speed=1):
        self.lock.acquire()
        x, y = self.simulator.grid_to_sim(position[0], position[1])
        side = self.simulator.side
        factor = side - side*0.3
        way_points = [ [x-factor/2, y+factor/2],
                       [x+factor/2, y+factor/2],
                       [x+factor/2, y-factor/2],
                       [x-factor/2, y-factor/2],
                       [x-factor/2, y+factor/2]
                     ]
        data = {}
        data["Request"] = "search"
        data["DestinationCoordinates"] = []
        for point in way_points:
            x,y = loc.LocalGrid2LatLong(point[0], point[1])
            data["DestinationCoordinates"].append([x,y])

        with open('data.json', 'w') as outfile:
            json.dump(data, outfile)
        send_email.send(data["Request"])
        self.lock.release()

    def get_tags(self, position):
       pass

    def searchComplete(self, destination):
        x,y = self.get_cell()
        print ("The agent is at: ", x,y)
        if [x,y] == destination:
            print ("The agent reached the destination")
            return True
        else:
            self.move_cell([0,0], destination)
            self.searchComplete(destination)

    def actual_search_complete(self, position):
        try:
            self.Recvlock.acquire()
            receive_email.recieve()
            completed = searchprocess()
            self.Recvlock.release()
            print ("search", completed)
            if completed:
                return True
            else:
                self.search(position)
                self.actual_search_complete(position)
        except Exception as e:
            self.Recvlock.release()
            print(e)

    def get_cell(self):
        try:
            self.Recvlock.acquire()
            data = {}
            #data["Request"] = "agent_location"
            #with open('data.json', 'w') as outfile:
            #    json.dump(data, outfile)
            #send_email.send(data["Request"])
            receive_email.recieve()
            lat, long, timestamp = process()
            x,y = loc.LatLong2LocalGrid(lat,long)
            x,y = self.simulator.sim_to_grid(x,y)

            self.Recvlock.release()
            return x,y

        except Exception as e:
            self.Recvlock.release()
            print(e)


    def getAdjacent(self, position):
        pass


def test():
        tag = TagWorld()
        start_time = time.time()

        #tag.move_cell([0,0], [1,2])
        #time.sleep(60)
        #tag.searchComplete([1,2])
        #print ("Time elapsed", (time.time() - start_time))

        #tag.search([1,2])
        tag.actual_search_complete([1,2])
        
        #tag.move_cell([0,1], [0,2])
        #time.sleep(120)
        #tag.searchComplete([0,2])
        print ("Time elapsed", (time.time() - start_time))
    
        #tag.search([0,2])
        """
        tag.actual_search_complete()
        print ("Time elapsed", (time.time() - start_time))
        """
#def test1():





if __name__ == "__main__":

        #tag = TagWorld()
        #x,y = tag.simulator.sim_to_grid(3200.98610615,-3200.00480807)
        #print (x,y)
        #tag.search([0,2])
        #tag.move_cell([0,0], [0,1])
        test()
        """
        start_time = 1616695200#
        current_time = time.time()
        #receive_email.recieve()
        lat, long, timestamp = process()
        x,y = loc.LatLong2LocalGrid(lat,long)
        x,y = tag.simulator.sim_to_grid(x,y)
        print ("The location is: ", x,y)
        print ("The predicted time stamp", lat, long, timestamp)
        predicted_time = timestamp
        elapsed_time = start_time + (time.time() - current_time)
        print ("Current Time", elapsed_time)
        while (elapsed_time < predicted_time):
            elapsed_time = start_time + (time.time() - current_time)
            pass

        x,y = loc.LatLong2LocalGrid(lat,long)

        tag.get_cell()
        receive_email.recieve()
        lat, long, timestamp = process()
        x,y = loc.LatLong2LocalGrid(lat,long)
        print ("The location is: ", x,y)
        print ("The predicted time stamp", lat, long, timestamp)
        tag.move_cell([0,0], [0,2])



        #    print (time.time() - start_time)

        #tag.get_cell()
        #tag.get_cell()
        """
