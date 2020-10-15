from __future__ import division
import math


class Simulator:

    def __init__(self, start_x=-141, start_y=76, side=80, size=5):
        """
        start_x : # starting x position in the moos Simulator
        start_y: # starting y position in the moos Simulator
        side : # side length of a grid cell
        size : grid size (5*5)
        """
        self.start_x = start_x
        self.start_y = start_y
        self.side = side
        self.size = size


    def sim_to_grid(self, x, y):

        cell_x = (self.start_x - x)/self.side
        cell_y = (self.size -1) - ((self.start_y - y)/self.side)
        return abs(int(math.ceil(cell_x))), abs(int(math.ceil(cell_y)))

    def grid_to_sim(self, x,y):


        y = self.size-1 - y

        grid_start_x = self.start_x + x*80
        grid_end_x = self.start_x + (x+1)*80

        grid_start_y = self.start_y - (y)*80
        grid_end_y = self.start_y - (y+1)*80

        x = (grid_start_x + grid_end_x)/2
        y = (grid_start_y + grid_end_y)/2

        return x,y
