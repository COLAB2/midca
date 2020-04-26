from numpy import random
import numpy as np
from midca.domains.moos_domain import moosworld
import zmq, time

def experimental_setup(experiments = 20, start = 0, choose = 20):

    ga1 = ["line", "random", "single"]
    ga1_prob = [0.25, 0.65, 0.1]
    ga2 = ["line", "random", "single"]
    ga2_prob = [0.25, 0.65, 0.1]
    qroute = ["single_line", "multiple_lines", "random", "multiple_random", "single_mine", "multiple_single_mine"]
    qroute_prob = [0.1, 0.1, 0.2, 0.5, 0.05, 0.05]
    transit = ["line", "random",  "single"]
    transit_prob = [0.25, 0.65, 0.1]
    qroute1 = ["none",  "random", "heavy", "multiple"]
    qroute1_prob = [0.1,  0.3, 0.25, 0.35]

    random.seed(0)
    ga1_mines = random.choice(ga1, experiments, p = ga1_prob)
    ga2_mines = random.choice(ga2, experiments, p = ga2_prob)
    qroute_mines = random.choice(qroute, experiments, p = qroute_prob)
    transit_mines = random.choice(transit, experiments, p = transit_prob)
    qroute1_mines = random.choice(qroute1, experiments, p = qroute1_prob)

    ga1_mines = ga1_mines[start:choose]
    ga2_mines = ga2_mines[start:choose]
    qroute_mines = qroute_mines[start:choose]
    transit_mines = transit_mines[start:choose]
    qroute1_mines = qroute1_mines[start:choose]

    """
    with open("experiments.csv", "a") as f:
            f.write ("Transit,GA1,GA2,Qroute,Qroute1\n")
            for index, each in enumerate(ga1_mines):
                f.write (transit_mines[index]  + "," + ga1_mines[index] + "," + ga2_mines[index] + "," + qroute_mines[index] + "," + qroute1_mines[index] + "\n")

    """
    return ga1_mines, ga2_mines, qroute_mines, transit_mines, qroute1_mines

class Experiment:

    def __init__(self, mean = [72, -53], cov=[[80, 0], [0, 80]], total_mines=10):
        # for Zmq
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher_mine = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5505")
        self.publisher_mine.bind("tcp://127.0.0.1:7593")

                # Mean and the covariance
        self.mean = mean
        self.cov = cov  # diagonal covariance
        self.total_mines = total_mines
        # we dont want to have the mines with the same lables
        self.label_count = 2000


    def send_message(self, x, y):
        # distribution of the x and the y
        for i, element in enumerate(x):
            content = [b"M", b"x=" + str(x[i]) + ",y=" + str(y[i]) + ",label= " + str(self.label_count) + ", type=hazard"]
            time.sleep(0.2)
            self.publisher.send_multipart(content)
            time.sleep(0.1)
            content_add = [b"M", b"hazard = x=" + str(x[i]) + ",y=" + str(y[i]) + ",label= " + str(self.label_count) + ", type=hazard"]
            self.publisher_mine.send_multipart(content_add)
            time.sleep(0.1)
            self.label_count +=1

    def lay_mines(self, index):
        ga1_mines, ga2_mines, qroute_mines, transit_mines, qroute1_mines = experimental_setup()

        print ("Ga1 {}".format(ga1_mines[index]))
        print ("Ga2 {}".format(ga2_mines[index]))
        print ("Qroute {}".format(qroute_mines[index]))
        print ("Transit {}".format(transit_mines[index]))
        print ("Qroute1 mines {}".format(qroute1_mines[index]))

        #qroute_mines[index] = "random"
        #ga1_mines[index] = "random"
        # ga1
        if ga1_mines[index] == "line":
            x = [6, 31, 18.5, 12.25, 9.125, 7.5625]
            y = [-69.2, -86.7, -77.95, -73.575, -71.3875, -70.29475]
            self.send_message(x,y)

        elif ga1_mines[index] == "random":
            x, y = np.random.multivariate_normal([16, -82], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif ga1_mines[index] == "single":
            x = [6]
            y = [-85]
            self.send_message(x,y)

        else:
            pass

        # ga2
        if ga2_mines[index] == "line":
            x = [137, 162, 149.5, 143.25, 140.125, 138.5625]
            y = [-65.9, -83.4, -74.65, -70.275, -68.0875, -66.99375]
            self.send_message(x,y)

        elif ga2_mines[index] == "random":
            x, y = np.random.multivariate_normal([148,-80], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif ga2_mines[index] == "single":
            x = [138]
            y = [-77]
            self.send_message(x,y)

        #qroute
        if qroute_mines[index] == "single_line":
            x = [45, 70, 57, 51, 48.125, 46.5625, 45, 63, 73, 80]
            x = [a for pair in zip(x,x) for a in pair]
            y = [-60.5, -83, -71.75, -66.125, -63.31, -61.9, -61.2, -77, -89, -95]
            y = [a for pair in zip(y,y) for a in pair]

            self.send_message(x,y)

        elif qroute_mines[index] == "multiple_lines":
            x = [45, 70, 57, 51, 48.125, 46.5625, 45, 63, 73, 80 , 100, 150, 125, 112, 106, 103, 101, 117]
            y = [-60.5, -83, -71.75, -66.125, -63.31, -61.9, -61.2, -77, -89, -95, -50, -95, -72, -61.25, -55, -52.8, -51.4, -65]
            x = [a for pair in zip(x,x) for a in pair]
            y = [a for pair in zip(y,y) for a in pair]
            self.send_message(x,y)

        elif qroute_mines[index] == "random":
            moosworld.random = True

        elif qroute_mines[index] == "multiple_random":
           moosworld.random = True
           moosworld.multiple_random = True

        elif qroute_mines[index] == "single_mine":
            moosworld.single = True

        elif qroute_mines[index] == "multiple_single_mine":
            moosworld.single = True
            moosworld.multiple_single = True
        else:
            pass


        #qroute1
        if qroute1_mines[index] == "none":
            pass

        elif qroute1_mines[index] == "random":
            x, y = np.random.multivariate_normal([143, -196], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif qroute1_mines[index] == "heavy":
            x, y = np.random.multivariate_normal([143, -196], self.cov, (self.total_mines + 30)).T
            self.send_message(x,y)

        elif qroute1_mines[index] == "multiple":
            x, y = np.random.multivariate_normal([143, -196], self.cov, self.total_mines).T
            self.send_message(x,y)
            x, y = np.random.multivariate_normal([17, -202], self.cov, (self.total_mines - 7)).T
            self.send_message(x,y)

        else:
            pass



        # transit
        if transit_mines[index] == "line":
            x = [9, 21.5, 15.25, 12.125, 10.5625, 9.78125]
            y = [-6.3, -15.05, -10.675, -8.4875, -7.39375, -6.846875]
            self.send_message(x,y)

        elif transit_mines[index] == "random":
            x, y = np.random.multivariate_normal([22, -1], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif transit_mines[index] == "single":
            x = [22]
            y = [-1]
            self.send_message(x,y)
        else:
            pass

class MultiExperiment:

    def __init__(self, mean = [72, -53], cov=[[80, 0], [0, 80]], total_mines=10):
        # for Zmq
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher_mine = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5505")
        self.publisher_mine.bind("tcp://127.0.0.1:7593")

                # Mean and the covariance
        self.mean = mean
        self.cov = cov  # diagonal covariance
        self.total_mines = total_mines
        # we dont want to have the mines with the same lables
        self.label_count = 2000


    def send_message(self, x, y):
        # distribution of the x and the y
        for i, element in enumerate(x):
            content = [b"M", b"x=" + str(x[i]) + ",y=" + str(y[i]) + ",label= " + str(self.label_count) + ", type=hazard"]
            time.sleep(0.2)
            self.publisher.send_multipart(content)
            time.sleep(0.1)
            content_add = [b"M", b"hazard = x=" + str(x[i]) + ",y=" + str(y[i]) + ",label= " + str(self.label_count) + ", type=hazard"]
            self.publisher_mine.send_multipart(content_add)
            time.sleep(0.1)
            self.label_count +=1

    def lay_mines(self, index):
        ga1_mines, ga2_mines, qroute_mines, transit_mines, qroute1_mines = experimental_setup()

        print ("Ga1 {}".format(ga1_mines[index]))
        print ("Ga2 {}".format(ga2_mines[index]))
        print ("Qroute {}".format(qroute_mines[index]))
        print ("Transit {}".format(transit_mines[index]))
        print ("Qroute1 mines {}".format(qroute1_mines[index]))

        #qroute_mines[index] = "random"
        #ga1_mines[index] = "random"
        # ga1
        if ga1_mines[index] == "line":
            x = [6, 31, 18.5, 12.25, 9.125, 7.5625]
            y = [-69.2, -86.7, -77.95, -73.575, -71.3875, -70.29475]
            self.send_message(x,y)

        elif ga1_mines[index] == "random":
            x, y = np.random.multivariate_normal([-26, -130], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif ga1_mines[index] == "single":
            x = [-26]
            y = [-130]
            self.send_message(x,y)

        else:
            pass

        # ga2
        if ga2_mines[index] == "line":
            x = [137, 162, 149.5, 143.25, 140.125, 138.5625]
            y = [-65.9, -83.4, -74.65, -70.275, -68.0875, -66.99375]
            self.send_message(x,y)

        elif ga2_mines[index] == "random":
            x, y = np.random.multivariate_normal([137,-129], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif ga2_mines[index] == "single":
            x = [137]
            y = [-129]
            self.send_message(x,y)

        #qroute
        if qroute_mines[index] == "single_line":
            x = [45, 70, 57, 51, 48.125, 46.5625, 45, 63, 73, 80]
            x = [a for pair in zip(x,x) for a in pair]
            y = [-60.5, -83, -71.75, -66.125, -63.31, -61.9, -61.2, -77, -89, -95]
            y = [a for pair in zip(y,y) for a in pair]

            self.send_message(x,y)

        elif qroute_mines[index] == "multiple_lines":
            x = [45, 70, 57, 51, 48.125, 46.5625, 45, 63, 73, 80 , 100, 150, 125, 112, 106, 103, 101, 117]
            y = [-60.5, -83, -71.75, -66.125, -63.31, -61.9, -61.2, -77, -89, -95, -50, -95, -72, -61.25, -55, -52.8, -51.4, -65]
            x = [a for pair in zip(x,x) for a in pair]
            y = [a for pair in zip(y,y) for a in pair]
            self.send_message(x,y)

        elif qroute_mines[index] == "random":
            moosworld.random = True

        elif qroute_mines[index] == "multiple_random":
           moosworld.random = True
           moosworld.multiple_random = True

        elif qroute_mines[index] == "single_mine":
            moosworld.single = True

        elif qroute_mines[index] == "multiple_single_mine":
            moosworld.single = True
            moosworld.multiple_single = True
        else:
            pass


        #qroute1
        if qroute1_mines[index] == "none":
            pass

        elif qroute1_mines[index] == "random":
            x, y = np.random.multivariate_normal([131, -283], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif qroute1_mines[index] == "heavy":
            x, y = np.random.multivariate_normal([131, -283], self.cov, (self.total_mines + 30)).T
            self.send_message(x,y)

        elif qroute1_mines[index] == "multiple":
            x, y = np.random.multivariate_normal([131, -283], self.cov, self.total_mines).T
            self.send_message(x,y)
            x, y = np.random.multivariate_normal([43, -276], self.cov, (self.total_mines - 7)).T
            self.send_message(x,y)

        else:
            pass



        # transit
        if transit_mines[index] == "line":
            x = [9, 21.5, 15.25, 12.125, 10.5625, 9.78125]
            y = [-6.3, -15.05, -10.675, -8.4875, -7.39375, -6.846875]
            self.send_message(x,y)

        elif transit_mines[index] == "random":
            x, y = np.random.multivariate_normal([22, -1], self.cov, self.total_mines).T
            self.send_message(x,y)

        elif transit_mines[index] == "single":
            x = [22]
            y = [-1]
            self.send_message(x,y)
        else:
            pass

experimental_setup()
