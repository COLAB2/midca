from __future__ import division
import zmq
import time
from simulator import Simulator
import threading
import numpy as np
import anomaly
import geniosDataProcess
import operator
import os
from startingPositions import positions

np.random.seed(555)

class TagWorld():

    def __init__(self, sub_ip = "tcp://127.0.0.1:4999",
                 sub_mine_ip = "tcp://127.0.0.1:4998",
                 pub_ip = "tcp://127.0.0.1:5999", name="grace", start_index = 0 ):

        context = zmq.Context()
        self.subscriber_remus = context.socket(zmq.SUB)
        self.subscriber_remus.setsockopt(zmq.SUBSCRIBE,  b'')
        self.subscriber_remus.setsockopt(zmq.RCVTIMEO, -1)
        self.subscriber_remus.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_remus.connect(sub_ip)

        context = zmq.Context()
        self.subscriber_mine = context.socket(zmq.SUB)
        self.subscriber_mine.setsockopt(zmq.SUBSCRIBE,  b'')
        self.subscriber_mine.setsockopt(zmq.RCVTIMEO, 3)
        self.subscriber_mine.connect(sub_mine_ip)


        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect(pub_ip)

        # initialize simulator
        self.name = name
        if name == "grace" or name == "franklin" or name == "remus":
            self.simulator = Simulator(start_x=-141, start_y=76, side=80, size_m=5, size_n = 5)
        else:
            self.simulator = Simulator(start_x=0, start_y=0, side=800, size_m=8, size_n = 11)

        #search complete
        self.searching = "false"
        self.speed = 0
        self.position = None

        #mines
        self.mines = {}
        self.mines_history = []

        # lock for threading
        self.lock = threading.Lock()
        self.Recvlock = threading.Lock()

        # initialize anomaly chance
        self.anomalies = False
        self.anomaly_type = ""
        self.anomalies_d = True

        #for report
        self.anomaly_history = []
        self.anomaly_time = []

        self.blown_out = False

        # move to starting location
        #self.initial_location(positions[start_index])

        #time
        self.experiment_time = 250
        self.time = 0
        self.start_time = self.set_initial_time()

        if self.anomalies_d:
            self.discrepency = anomaly.Anomalies(self.time, 0.5, 555, self.simulator, self.publisher)
            #threading.Thread(target=self.exit).start()
            threading.Thread(target=self.attacks).start()
            #anomalies.start()

        #hack
        self.timeElapsed = False
        self.timeElapsedM3 = False

        self.mode = "m1"
        #self.startsim()

    def initial_location(self, position):

        x,y = position[0], position[1]
        # construct the message
        message = [b"Vehicle", b"point = "+str(x)+","+str(y)+" # speed= 1.0"]

        time.sleep(0.1)
        # send the message
        self.publisher.send_multipart(message)

        time.sleep(1)

        while ( self.searchComplete() == str(False) ):
            # wait for the vehicle to move to starting point
            pass


    def exit(self):
        while True:
            time_elapsed = (time.time() - self.start_time)
            if ( time_elapsed >= self.experiment_time):
                self.report()
                os._exit(1)

    def bin(self, key):
        temp = key.split(",")
        temp = [int(each) for each in temp]
        return temp[1]*5 + temp[0]


    def report(self):
        self.lock.acquire()

        hotspots = []
        hotspot_count = []

        #prepare report
        for key, value in self.mines.iteritems():
            if len(value) > 40:
                hotspots.append(self.bin(key))
                hotspot_count.append(len(value))

        f = open( "log.csv", 'a')
        position = ";".join(str(row) for row in hotspots)
        hotspot_count = ";".join(str(row) for row in hotspot_count)
        anomaly_history = ";".join(str(row) for row in self.anomaly_history)
        anomaly_time = [round(each) for each in self.anomaly_time if each]
        anomaly_time_history = ";".join(str(row) for row in anomaly_time)
        #print (str(len(hotspots)) + str(position) + "," + str(len(anomaly_history)) + str(anomaly_history) + "," + str(anomaly_time_history)  +"\n")
        f.write(str(len(hotspots)) + "," + str(position) + "," + str(hotspot_count)
                + "," + str(len(self.anomaly_history)) + "," + str(anomaly_history)
                + "," + str(anomaly_time_history)  +"\n")
        f.close()

        self.lock.release()




    def TagWorld(self):
        return self

    def startsim(self):
        # construct the message
        loc = self.get_cell()
        loc = loc.split(",")
        self.move_cell([0,0], [int(loc[0]) , int(loc[1])])

    def endSim(self):
        pass

    def get_mode(self):
        if self.mode == "m3":
            return "m3"

        if self.searchComplete() == str(False) and self.speed < 0.7 and self.speed >=0.0:
            if self.timeElapsed:
                wait_time = time.time() - self.timeElapsed
                if (wait_time) > 0:
                    self.timeElapsedM3 = False
                    return "m2"
            else:
                self.timeElapsed = time.time()

        elif self.speed < 0.1:
            if self.timeElapsedM3:
                wait_time = time.time() - self.timeElapsedM3
                if (wait_time) > 30:
                    self.mode = "m3"
                    return "m3"
            else:
                self.timeElapsedM3 = time.time()
        else:
            self.timeElapsed = False
            self.timeElapsedM3 = False
            return "m1"

    def move_cell(self, initial_position, final_destination):
        self.lock.acquire()

        time.sleep(0.25)

        position = None
        if self.anomalies_d:
            position, discrepency = self.discrepency.identify_anomaly(initial_position[0], initial_position[1],
                                          final_destination[0], final_destination[1], self.time)
            if discrepency:
                self.anomaly_type = discrepency
                self.anomaly_history.append(discrepency)
                time_elapsed = (time.time() - self.start_time)
                self.anomaly_time.append(time_elapsed)
            else:
                self.anomaly_type = ""

        if position:
            x,y = position[0], position[1]
        else:
            #get the real moos coordinate equivalent of cell coordinates
            x,y = self.simulator.grid_to_sim(final_destination[0], final_destination[1])

        # construct the message
        message = [b"Vehicle", b"point = "+str(x)+","+str(y)+" # speed= 1.0"]

        # send the message
        self.publisher.send_multipart(message)

        time.sleep(0.05)

        self.lock.release()

    def search(self, position, speed=1):
        self.lock.acquire()
        if self.blown_out:
            time.sleep(2)
            return

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

        self.lock.release()

    def send(self, way_points, speed=1):
        self.lock.acquire()
        time.sleep(0.25)

        points = b"points = pts={"

        for point in way_points:
            points = points + str(point[0]).encode('ascii') + b"," + str(point[1]).encode('ascii') + b":"

        # remove the column(:) from the end of the points string and add flower braces
        points = points[:-1] + b"}"

        # create message
        message = [b"Vehicle", points + b"# speed= "+str(speed).encode('ascii')]

        self.publisher.send_multipart(message)

        time.sleep(0.25)

        self.lock.release()

    def deepsearch(self, position, speed = 0.8):
        self.lock.acquire()

        time.sleep(0.25)

        x, y = self.simulator.grid_to_sim(position[0], position[1])

        # create message
        message = [b"Vehicle",b" points=format=lawnmower,label=dedley_survey, x="+str(x)+", y="+str(y)+", width=60, height = 70,lane_width=10, rows=north-south,degs=0 # speed =0.8"]

        self.publisher.send_multipart(message)

        time.sleep(0.25)

        self.lock.release()

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
            time = float(time.split("=")[1]) #- self.start_time
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
            print ('hit duplicate mine return...')
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
        if self.searching == "true":
            return str(False)
        elif self.anomaly_type:
            return "Failed"
        else:
            return str(True)

    def get_cell(self):

        try:
            self.Recvlock.acquire()
            # get the message from the ip address
            position = self.subscriber_remus.recv()
            x, y, speed, direction,status,mission,time,name = position.split(",")

            # convert to midca coordinates
            # the content is of the form "X:-101.008305,Y:-44.089216,SPEED:0.000000,HEADING:4.079004,STATUS:NothingToDo"
            x = float(x.split(":")[1])
            y = float(y.split(":")[1])
            self.searching = mission.split(":")[1]
            self.time = float(time.split(":")[1])
            self.speed = float(speed.split(":")[1])
            self.position = x,y

            # get the grid coordinates
            x, y = self.simulator.sim_to_grid(x,y)
            self.Recvlock.release()
            return str(x)+","+str(y)

        except Exception as e:
            self.Recvlock.release()
            print(e)

    def kill_simulator(self):
        try:
            self.simulate_anomalies.kill()
        except:
            import sys
            sys.exit()

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
        #self.get_cell()
        return time.time()

    def get_time(self):
        self.get_cell()
        return self.time - self.start_time

    def attacks(self):
        flag = 0
        stop = False
        wreck = False
        speed = 0.5
        count = 1
        #switchprob = 1/100.0
        switchprob = 0.65/100
        if self.name == "franklin":
            switchprob = 100/100
        if self.name == "grace":
            switchprob = 1/100
        #else:
        #    breakprob = 0
        while True:

            self.lock.acquire()
            #flag = self.attack_blow(flag)

            temp = np.random.rand()
            if wreck:
                speed = 0.0
                if not stop:
                    # construct the message
                    message = [b"Vehicle", b"speed= "+str(speed)]

                    # send the message
                    self.publisher.send_multipart(message)

                    stop = True


            if self.anomalies:
                    if temp < switchprob:
                            count += 1
                            speed -= count * 0.2
                            #speed = 0.5
                            if speed < 0.2:
                                speed = 0.01

                    #if temp < breakprob:
                    #        wreck = True
                    if self.searchComplete() == str(False):

                        # construct the message
                        message = [b"Vehicle", b"speed= "+str(speed)]

                        # send the message
                        self.publisher.send_multipart(message)
                        time.sleep(0.1)
                        index = self.position[0] + self.position[1]
                        message = [b"AddMarker",
                           b"type=diamond,"
                           b"x="+str(self.position[0])+",y="+str(self.position[1])+","+
                            "scale=4,label=remora"+str(index)+",color=yellow,width=4.5"]
                        self.publisher.send_multipart(message)

                        #time.sleep(0.1)



            else:
                if temp < switchprob:
                    self.anomalies = True
                count = 0
                speed = 0.5
            self.lock.release()
            time.sleep(0.1)

    def attack_blow(self, flag=0):
        if flag == 1:
            time.sleep(4)
            # construct the message
            message = [b"Vehicle", b"speed=0"]
            # send the message
            self.publisher.send_multipart(message)
            time.sleep(0.1)
            self.publisher.send_multipart(message)
            time.sleep(0.1)
            self.publisher.send_multipart(message)
            os._exit(1)
            #return flag

        cell = self.get_cell()
        if cell == "0,1" and not flag:
           message = [b"Vehicle",b"points = format=ellipse, label=golf, x=-164, y=-199, degs=45, pts=7, snap=10,major=70,minor=100# speed =1"]
           time.sleep(0.1)
           self.publisher.send_multipart(message)
           flag = 1
           #time.sleep(2)
           self.blown_out = True



        return flag


    def remove_remora(self):
        self.lock.acquire()

        # construct the message
        message = [b"Vehicle", b"speed= 0"]
        # send the message
        self.publisher.send_multipart(message)

        time.sleep(2)

        self.lock.release()
        return True

    def remove_remora_status(self):
        self.lock.acquire()
        self.anomalies = False
        speed = 1.0
        self.timeElapsed = False

        if self.searchComplete() == str(False):
            # construct the message
            message = [b"Vehicle", b"speed= "+str(speed)]

            # send the message
            self.publisher.send_multipart(message)

        self.lock.release()

        return str(True)

    def get_anomaly(self):
        type = self.anomaly_type
        #self.anomaly_type = ""
        return type

    def inspect(self, initial_position, dest_position):
        time.sleep(4)
        [agent_x, agent_y] = initial_position
        [agent_dest_x, agent_dest_y] = dest_position
        x,y = self.simulator.grid_to_sim(agent_x, agent_y)
        x1, y1 = self.simulator.grid_to_sim(agent_dest_x, agent_dest_y)
        position = (x+x1)/2 , (y+y1)/2
        threshold = self.discrepency.get_threshold([initial_position, dest_position])
        position = map(operator.sub, position, threshold)
        if threshold[0] == 0:
            way_point0 = map(operator.add, position, [-30, 0])
            way_point1 = map(operator.add, position, [30, 0])
            self.send([way_point0, way_point1])
        else:
            way_point0 = map(operator.add, position, [0, -30])
            way_point1 = map(operator.add, position, [0, 30])
            self.send([way_point0, way_point1])
        self.anomaly_type = ""

    def get_hotspot(self, position):
        return 0

    def UpdateSurfstatus(self, status):
        return

    def timeRemaining(self):
        return (self.experiment_time - (time.time() - self.start_time))


if __name__ == "__main__":

        tag = TagWorld(name="grace")
        #tag.move_cell([1,4], [1,3])
        #way_points = geniosDataProcess.process()
        #tag.send(way_points)

        #print (way_points)
        #print (way_points)
        #x,y = -134,-267
        #message = [b"Vehicle",b"points = format=ellipse, label=golf, x=-164, y=-199, degs=45, pts=7, snap=10,major=70,minor=100# speed =2"]
        #time.sleep(0.1)
        #tag.publisher.send_multipart(message)
        #while(True):
        #    print (tag.get_cell())
        #tag.move_cell([0,0], [0,1])
        #print(tag.get_cell())
        #print (tag.get_time())
        #print(tag.deepsearch([0,0]))
        #while (tag.searchComplete() == "false"):
        #    print(tag.get_tags([0,0]))
        #    #tag.getAdjacent([0,0])
        #print(tag.getAdjacent([0,0]))
        #print (tag.get_tags([0,0]))
