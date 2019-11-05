import math
import threading
import time
import zmq

import mine_layer

pirate_flag = False

# mines set by experiment.py
random = False
multiple_random = False
single = False
multiple_single = False

# change qroute
change_qroute = []
for i in range(0,10):
    change_qroute.append(False)


class MoosWorld:
    """
    This class interfaces with moos
    """

    def __init__(self, friendly_delay, init_enemy=False, init_friendly_vessels=False):

        #delay until friendly vessels start moving
        self.delay = friendly_delay

        # initialize the socket connections
        self.declare_connections()

        # initialize the enemy behavior as a thread
        if init_enemy:
            enemy_laying_mines = threading.Thread(target=self.enemy_movement)

        if init_friendly_vessels:
            ship_traversal = threading.Thread(target=self.ship_movement)


        # execute the thread
        # This should probably be protected with an if init_enemy. DO
        # Q: thead line have to be executed before socket line? else can put
        # in same if block DO
        if init_enemy:
            enemy_laying_mines.daemon = True
            enemy_laying_mines.start()

        # waypoints for the friendly vessels
        self.way_points_friendly_vessels = []

        if init_friendly_vessels:
            ship_traversal.daemon = True
            ship_traversal.start()

    def declare_connections(self):
        """
        Establish the socket connections required with moos
        :return:
        """

        # Zeromq connection with the moos for perception
        context = zmq.Context()

        # Zeromq connection with the fisher (enemy) for action
        self.publisher_enemy = context.socket(zmq.PUB)
        self.publisher_enemy.connect("tcp://127.0.0.1:5592")

        # Zeromq connection with the fisher (enemy) for perception
        self.subscriber_enemy = context.socket(zmq.SUB)
        self.subscriber_enemy.setsockopt(zmq.SUBSCRIBE, '')
        self.subscriber_enemy.setsockopt(zmq.RCVTIMEO, 5)
        self.subscriber_enemy.setsockopt(zmq.CONFLATE, 1)
        self.subscriber_enemy.bind("tcp://127.0.0.1:6590")

        # Zeromq connection with the friendly vessels for action
        self.publisher_friendly_vessels = []
        for i in range(0,10):
            connection  = context.socket(zmq.PUB)
            connection.connect("tcp://127.0.0.1:652"+str(i))
            self.publisher_friendly_vessels.append(connection)
            time.sleep(0.1)

    # Current working assumption - takes the cell and sends a message to the
    # the simulation to move the enemy to the associated point  DO
    def apply_enemy_action(self, point):
        """
        :param cell: The location of the cell enemy should go to
        :return:
        """
        message = [b"M", b"point = " + str(point[0]) + "," + str(point[1]) + "# speed= 1.0"]
        time.sleep(0.1)
        self.publisher_enemy.send_multipart(message)
        time.sleep(0.1)
        self.publisher_enemy.send_multipart(message)

    def enemy_way_point_behavior(self, way_points):
        """
        :param way_point: List of way_points enemy should travel
        :return:
        """
        #so each is just the typical cell coordinate string, since
        # way_points is a list of such coordinates
        for each in way_points:
            #abort if pirate is captured
            if (pirate_flag == True):
                for i in range(2):
                    self.publisher_enemy.send_multipart([b"M", b"speed =0.0"])
                return
            #time.sleep(0.1)
            # So this sends the message to move to the waypoint... DO
            self.apply_enemy_action(each)
            x=0
            y=0
            # And so this loops until enemy ship arrives... DO
            # Corrected use of dangerous calculate_cell method DO
            #while not (self.calculate_cell(x, y) == each):
            while (x >= each[0] + 5 or x <= each[0] - 5) \
                        and (y > each[1] +5 or y <= each[1] - 5):
                try:
                    enemy_pos = self.subscriber_enemy.recv()
                    x, y, speed, direction = enemy_pos.split(",")
                    x = float(x.split(":")[1])
                    y = float(y.split(":")[1])
                except:
                    pass


    def enemy_movement(self):
        """
        The enemy movement and mine laying activity
        :return:
        """
        time.sleep(5)
        layer = None
        # changed waypoint to waypoints DO
        #way_points = ["c2.19", "c3.19", "c4.19", "c4.18", "c4.17", "c4.16", "c5.16", "c5.15", "c5.14",
             #"c5.13", "c5.12", "c5.11", "c5.10"]
        global pirate_flag, random, multiple_random, single, multiple_single
        #abort if pirate is captured
        if (pirate_flag == True):
            for i in range(2):
                self.publisher_enemy.send_multipart([b"M", b"speed =0.0"])
            return

        if single:
            way_points = [[57, -73]]
            self.enemy_way_point_behavior(way_points)
            layer = mine_layer.Minelayer(mean = way_points[0], cov=[[100, 0], [0, 100]], total_mines=1)
            layer.send_message()
            way_points = [[245,42]]
            self.enemy_way_point_behavior(way_points)

        if multiple_single:
            way_points = [[111, -73]]
            self.enemy_way_point_behavior(way_points)
            layer.set_mean(way_points[0])
            layer.send_message()
            way_points = [[245,42]]
            self.enemy_way_point_behavior(way_points)
            return

        if random:
            way_points = [[57, -73]]
            self.enemy_way_point_behavior(way_points)
            layer = mine_layer.Minelayer(mean = way_points[0], cov=[[80, 0], [0, 80]], total_mines=10)
            layer.send_message()
            way_points = [[245,42]]
            self.enemy_way_point_behavior(way_points)

        if multiple_random:
            way_points = [[111, -73]]
            self.enemy_way_point_behavior(way_points)
            layer.set_mean(way_points[0])
            layer.send_message()
            way_points = [[245,42]]
            self.enemy_way_point_behavior(way_points)
            return

        """
        # lay mines
        # ideally would have more realistic way of "mining an area" with a
        # Guassian distribution DO
        layer = mine_layer.Minelayer(way_points[0])
        layer.send_message()

        # 2nd Guassian Code
        way_points = [[245,42], [111, -73]]
        self.enemy_way_point_behavior(way_points)

        #abort if pirate is captured
        if (pirate_flag == True):
            for i in range(2):
                self.publisher_enemy.send_multipart([b"M", b"speed =0.0"])
            return

        mean = way_points[1]
        layer.set_mean(mean)
        layer.send_message()

        # 3rd Guassian
        way_points = [[245,42]]
        self.enemy_way_point_behavior(way_points)
        """

    def ship_way_points_behavior(self, publisher, way_points, speed=0.5):
        """
        :param publisher : publisher associated for a ship
        :param way_point: List of way_points enemy should travel
        :return:
        """
        #so each is just the typical cell coordinate string, since
        # way_points is a list of such coordinates
        # points variable is to accumulate all the points to send
        # example repr of points  "points = pts={60,-40:60,-160:150,-160:180,-100:150,-40}"
        points = b"points = pts={"
        for point in way_points:
            points = points + str(point[0]) + "," + str(point[1]) + ":"

        # remove the column from the end of the points string and add flower braces
        points = points[:-1] + "}"

        # create message
        message = [b"M", points + "# speed= "+str(speed)]

        # send the message
        time.sleep(0.1)
        publisher.send_multipart(message)
        time.sleep(0.1)
        publisher.send_multipart(message)


    def ship_movement(self):
        """
        Ships movement
        :return:
        """
        # please adjust the sleep for the delay
        #time.sleep(self.delay)

        global change_qroute

        self.way_points_friendly_vessels = [
                                                # Qroute 1
                                                [
                                                [[-39, -52], [227, -62]],
                                                [[-39, -70], [217, -86]],
                                                [[-39, -90], [228, -74 ]],
                                                [[-39, -80], [217, -86]],
                                                [[-39, -90], [217, -84]],
                                                [[-39, -60], [217, -55]],
                                                [[-39, -55], [225, -65]],
                                                [[-39, -93], [225, -69]],
                                                [[-39, -75], [225, -75]],
                                                [[-39, -54], [218, -93]],
                                                ],

                                                #Qroute 2
                                                [
                                                [[-119, -173], [219, -222]],
                                                [[-120, -182], [218, -216]],
                                                [[-119, -221], [210, -174 ]],
                                                [[-119, -210], [217, -190]],
                                                [[-119, -212], [217, -207]],
                                                [[-119, -218], [217, -185]],
                                                [[-119, -200], [217, -180]],
                                                [[-119, -190], [217, -195]],
                                                [[-119, -180], [217, -178]],
                                                [[-119, -195], [218, -195]],
                                                ]

                                            ]

        # this is to publish the way_points 1 is the speed
        speed = 1
        change = 0
        for i in range(0, len(self.publisher_friendly_vessels)):
            if (i%3 == 0):
                time.sleep(5)

            if change_qroute[i]:
                    change = 1
            self.ship_way_points_behavior(self.publisher_friendly_vessels[i],
                                          self.way_points_friendly_vessels[change][i], speed)
            time.sleep(0.2)

def main(friendly_delay):
    w = MoosWorld(friendly_delay, True, True)

# this must go at end otherwise main method does not see helper functions
if __name__ == '__main__':
    main(0)
    pass
