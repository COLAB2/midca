from __future__ import division
import time
from simulator import Simulator
import threading
import numpy as np
import loc
import json
import send_email, receive_email

np.random.seed(555)

class TagWorld():

    def __init__(self, name="genios" ):

        # initialize simulator
        self.name = name
        self.simulator = Simulator(start_x=0, start_y=0, side=800, size_m=8, size_n = 11)

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

    def searchComplete(self):
        pass

    def get_cell(self):
        try:
            self.Recvlock.acquire()
            data = {}
            data["Request"] = "agent_location"
            with open('data.json', 'w') as outfile:
                json.dump(data, outfile)
            send_email.send(data["Request"])
            #receive_email.recieve()

        except Exception as e:
            self.Recvlock.release()
            print(e)
        """
        try:
            self.Recvlock.acquire()
            # get the message from the ip address
            position = self.subscriber_remus.recv()
            x, y, speed, direction,status,mission,time = position.split(",")

            # convert to midca coordinates
            # the content is of the form "X:-101.008305,Y:-44.089216,SPEED:0.000000,HEADING:4.079004,STATUS:NothingToDo"
            x = float(x.split(":")[1])
            y = float(y.split(":")[1])
            self.searching = mission.split(":")[1]
            self.time = float(time.split(":")[1])
            self.speed = float(speed.split(":")[1])

            # get the grid coordinates
            x, y = self.simulator.sim_to_grid(x,y)
            self.Recvlock.release()
            return str(x)+","+str(y)

        except Exception as e:
            self.Recvlock.release()
            print(e)
        """


    def getAdjacent(self, position):
        pass



if __name__ == "__main__":

        tag = TagWorld()
        tag.search([0,1])
        #tag.get_cell()
        #tag.get_cell()
