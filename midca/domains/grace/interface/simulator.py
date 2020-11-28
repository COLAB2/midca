from __future__ import division
import math


class Simulator:

    def __init__(self, start_x=0, start_y=0, side=800, size_m=11, size_n = 8):
        """
        start_x : # starting x position in the moos Simulator
        start_y: # starting y position in the moos Simulator
        side : # side length of a grid cell
        size_m, size_n : grid size (5*5)
        anomalies : remora attacks
        """
        self.start_x = start_x
        self.start_y = start_y
        self.side = side
        self.size_m = size_m
        self.size_n = size_n


    def sim_to_grid(self, x, y):

        cell_x = (self.start_x - x)/self.side
        cell_y = (self.size_m -1) - ((self.start_y - y)/self.side)
        x = abs(int(math.ceil(cell_x)))
        y = abs(int(math.ceil(cell_y)))
        if x >=5:
            x =4
        if y >=5:
            y = 4
        return x, y

    def grid_to_sim(self, x,y):


        y = self.size_m-1 - y

        grid_start_x = self.start_x + x*self.side
        grid_end_x = self.start_x + (x+1)*self.side

        grid_start_y = self.start_y - (y)*self.side
        grid_end_y = self.start_y - (y+1)*self.side

        x = (grid_start_x + grid_end_x)/2
        y = (grid_start_y + grid_end_y)/2

        return x,y

if __name__ == "__main__":
    sim = Simulator(start_x=0, start_y=0, side=800, size_m=8, size_n = 11)
    print (sim.grid_to_sim(0,0))
    print (sim.sim_to_grid(8335, -6039))
