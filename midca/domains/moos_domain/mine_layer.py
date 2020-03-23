import numpy as np
import zmq
import time


class Minelayer:

    def __init__(self, mean = [72, -53], cov=[[100, 0], [0, 100]], total_mines=10):
        # for Zmq
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher_mine = context.socket(zmq.PUB)
        self.publisher.connect("tcp://127.0.0.1:5505")
        self.publisher_mine.bind("tcp://127.0.0.1:5522")

        # Mean and the covariance
        self.mean = mean
        self.cov = cov  # diagonal covariance
        self.total_mines = total_mines
        # we dont want to have the mines with the same lables
        self.label_count = 0 # for the mine labels

    def send_message(self):
        print 'total_mines =', self.total_mines
        print '----------------------'
        # distribution of the x and the y
        x, y = np.random.multivariate_normal(self.mean, self.cov, self.total_mines).T
        for i, element in enumerate(x):
            time.sleep(0.2)
            content = [b"M", b"x=" + str(x[i]) + ",y=" + str(y[i]) + ",label= " + str(self.label_count) + ", type=hazard"]
            self.publisher.send_multipart(content)
            time.sleep(0.1)
            content_add = [b"M", b"hazard = x=" + str(x[i]) + ",y=" + str(y[i]) + ",label= " + str(self.label_count) + ", type=hazard"]
            self.publisher_mine.send_multipart(content_add)
            time.sleep(0.1)
            self.label_count +=1

    # Method added by DO
    # change the mean so that pirate ship can move to a new location and lay
    # mines according to a new mean
    def set_mean(self, mean = [72, -53]):
        self.mean = mean
