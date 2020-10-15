from __future__ import division
import zmq
import time
from simulator import Simulator

class TagWorld():

    def __init__(self):
        context = zmq.Context()
        self.subscriber_remus = context.socket(zmq.SUB)
        self.subscriber_remus.setsockopt(zmq.SUBSCRIBE, "")
        self.subscriber_remus.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber_remus.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_remus.connect("tcp://127.0.0.1:4999")

        context = zmq.Context()
        self.subscriber_mine = context.socket(zmq.SUB)
        self.subscriber_mine.setsockopt(zmq.SUBSCRIBE, "")
        self.subscriber_mine.setsockopt(zmq.RCVTIMEO, 3)
        self.subscriber_mine.connect("tcp://127.0.0.1:4998")


        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5999")

        # initialize simulator
        self.simulator = Simulator()

        #search complete
        self.searching = "false"

        #time
        self.time = 0
        self.start_time = self.set_initial_time()

        #mines
        self.mines = {}
        self.mines_history = []

    def TagWorld(self):
        return self

    def endSim(self):
        pass

    def get_mode(self):
        return "m1"

    def move_cell(self, initial_position, final_destination):
        time.sleep(0.25)

        #get the real moos coordinate equivalent of cell coordinates
        x,y = self.simulator.grid_to_sim(final_destination[0], final_destination[1])

        # construct the message
        message = [b"Vehicle", b"point = "+str(x)+","+str(y)+" # speed= 1.0"]

        # send the message
        self.publisher.send_multipart(message)

    def search(self, position, speed=1):
        time.sleep(0.25)

        x, y = self.simulator.grid_to_sim(position[0], position[1])
        side = self.simulator.side
        factor = side - side*0.3
        way_points = [ [x-factor/2, y+factor/2],
                       [x+factor/2, y+factor/2],
                       [x+factor/2, y-factor/2],
                       [x-factor/2, y-factor/2],
                       [x-factor/2, y+factor/2]
                     ]

        points = b"points = pts={"

        for point in way_points:
            points = points + str(point[0]) + "," + str(point[1]) + ":"

        # remove the column(:) from the end of the points string and add flower braces
        points = points[:-1] + "}"

        # create message
        message = [b"Vehicle", points + "# speed= "+str(speed)]

        self.publisher.send_multipart(message)

        time.sleep(0.25)

    def check_if_new_mine(self, mine, verbose=1):
        """
        :param mine: instance of the Mine class
        :return: boolean : checks if it is the new mine
        """
        # Begin new method DO
        #Added base case of 0 mines... DO
        if len(self.world.mines) == 0:
            return True

        #End added code DO


        #why not use a dict for faster lookup DO?
        for each in self.world.mines:
            if each.label == mine.label:
                #return True
                # corrected logic - changed return to False DO
                return False

        #return False
        #corrected logic - changed return to True  DO
        return True

    def update_mines(self):
        """

        :return: Updates the dictionary if there are any mines
        """

        #Attempting to add a return value that signals if a mine was detected DO
        # 0 - no mine added
        # 1 = mine added
        #ret = 0
        try:
            mine_report = self.subscriber_mine.recv()
            mine_x, mine_y, mine_label,time = mine_report.split(":")[1].split(",")
            mine_x = float(mine_x.split("=")[1])
            mine_y = float(mine_y.split("=")[1])
            mine_label = mine_label.split("=")[1]
            time = float(time.split("=")[1]) - self.start_time
            #print 'label of mine at x =', mine_x, 'y =', mine_y, 'is', mine_label, 'time', time

            if not mine_label in self.mines_history:
                self.mines_history.append(mine_label)
                x,y = self.simulator.sim_to_grid(mine_x, mine_y)
                key = str(x)+","+str(y)
                if key in self.mines:
                    self.mines[key].append([mine_x, mine_y, mine_label, time])
                else:
                    self.mines[key] = [[mine_x, mine_y, mine_label, time]]

                return 1
            #added return value DO
            # value should be 1 because we should try again even if we hit
            # duplicate mine -- maybe sim returns duplicate mines in this
            # for unknown technical reasons
            print 'hit duplicate mine return...'
            return 1
        except Exception as e:
            #begin added code DO
            # added warning DO
            #print 'No mines received'
            #print 'Warning - sent to exception code in update_mines'
            return 0
            #end added code DO
            pass
        #added safety return value DO
        return 0

    def get_tags(self, position):

        # update the mines
        #Adding while loop to detect all mines placed DO
        while (self.update_mines() == 1):
                pass

        key = ",".join([str(pos) for pos in position])
        if key in self.mines:
            return len(self.mines[key])
        else:
            return 0

    def searchComplete(self):
        self.get_cell()
        if self.searching == "false":
            return str(True)
        else:
            return str(False)

    def get_cell(self):
        try:
            # get the message from the ip address
            position = self.subscriber_remus.recv()
            x, y, speed, direction,status,mission,time = position.split(",")

            # convert to midca coordinates
            # the content is of the form "X:-101.008305,Y:-44.089216,SPEED:0.000000,HEADING:4.079004,STATUS:NothingToDo"
            x = float(x.split(":")[1])
            y = float(y.split(":")[1])
            self.searching = mission.split(":")[1]
            self.time = float(time.split(":")[1])

            # get the grid coordinates
            x, y = self.simulator.sim_to_grid(x,y)

            return str(x)+","+str(y)

        except Exception as e:
            print(e)

    def lambda_rate(self, time):
        """
        time_interval = 11
        count = []
        for i in range(1, len(time)):
            j = i-1
            keep_count = 1
            while (j>=0):
                difference = time[i] - time[j]
                if  difference <= time_interval:
                    keep_count += 1
                    j -= 1
                else:
                    break
            count.append(keep_count)

        print count/6
        """
        return sum(time)/6


    def getAdjacent(self, position):
        key = ",".join([str(pos) for pos in position])
        x,y = self.simulator.grid_to_sim(position[0], position[1])
        side = self.simulator.side/2
        factor = 0.6
        xll = [x - side*factor, x+ side*factor]
        yll = [y- side*factor, y + side*factor]

        count = [0, 0, 0, 0]
        time = [[], [], [], []]

        if key in self.mines:
            for data in self.mines[key]:
                if data[1] > (yll[1]):
                    #north
                    count[0] += 1
                    time[0].append(data[3])

                    pass
                if data[1] < (yll[0]):
                    # south
                    count[1] += 1
                    time[1].append(data[3])
                    pass
                if data[0] > (xll[1]):
                    #east
                    count[2] += 1
                    time[2].append(data[3])
                    pass
                if data[0] < (xll[0]):
                    #west
                    count[3] += 1
                    time[3].append(data[3])
                    pass


        print (time)
        print (count)
        result = [each/6 for each in count]

        return result

    def set_initial_time(self):
        self.get_cell()
        return self.time

    def get_time(self):
        self.get_cell()
        return self.time - self.start_time



if __name__ == "__main__":

        tag = TagWorld()
        #tag.move_cell([0,0], [0,1])
        tag.get_cell()
        #print (tag.get_time())
        print(tag.search([0,0]))
        while (tag.searchComplete() == "false"):
            print(tag.get_tags([0,0]))
            #tag.getAdjacent([0,0])
        print(tag.getAdjacent([0,0]))
        #print (tag.get_tags([0,0]))
