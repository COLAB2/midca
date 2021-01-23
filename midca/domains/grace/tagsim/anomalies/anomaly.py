import random
from gridworld import Grid
import numpy as np

class Anomalies:

    def __init__(self, simtime, time_step, seed, E):
        self.simtime = simtime
        self.row_dimension = 5
        self.col_dimension = 5
        self.time_step = time_step
        self.E = E
        random.seed(seed)
        self.flow_occurance = None
        self.blocked_cells = None
        self.time_anomaly = None
        self.flow_anomaly(60)
        self.block_anomaly(7)


    def throw_direction(self, x,y):
        choice = random.choice(["x","y"])
        choices_x = []
        choices_y = []
        if x-1 > 0 and x-1 < self.row_dimension:
            choices_x.append(x-1)
        if x+1 > 0 and x+1 < self.row_dimension:
            choices_x.append(x+1)
        if y-1 > 0 and y-1 < self.col_dimension:
            choices_y.append(y-1)
        if y+1 > 0 and y+1 < self.row_dimension:
            choices_y.append(y+1)
        if choice == "x":
            return random.choice(choices_x), y
        else:
            return x, random.choice(choices_y)


    def flow_anomaly(self, frequency, k=7):
        """
        :param time: time steps as an input
        :return:
        """
        # dictionary for maintaining time step to location
        flow_occurance = {}
        time_steps = [step*self.time_step for step in range(int(self.simtime/self.time_step))]
        rows = [i for i in range(1, self.row_dimension+1)]
        cols = [i for i in range(1, self.col_dimension+1)]
        self.time_anomaly = random.choice(time_steps)
        occur_anomaly = True
        x,y = 0,0
        dest_x, dest_y = 0,0
        for index, value in enumerate(time_steps):
            remainder = int(index)%int(frequency)
            if remainder == 0:
                cells = []
                position_cells = []
                i = k
                while(i):
                    x = random.choice(rows)
                    y = random.choice(cols)
                    dest_x , dest_y = self.throw_direction(x,y)
                    if not (([x,y]  in position_cells) or ([dest_x,dest_y] in position_cells)):
                        cells.append([[x,y], [dest_x, dest_y]])
                        position_cells.append([x,y])
                        i -=1
                #occur_anomaly = not(occur_anomaly)

            if occur_anomaly:
                flow_occurance[value] = cells

        self.flow_occurance = flow_occurance

    def block_anomaly(self, number_block_cells):
        blocked_cells = {}
        cells = []
        position_cells = []
        rows = [i for i in range(1, self.row_dimension+1)]
        cols = [i for i in range(1, self.col_dimension+1)]
        i = number_block_cells
        while(i):
            x = random.choice(rows)
            y = random.choice(cols)
            dest_x , dest_y = self.throw_direction(x,y)
            if not [[x,y], [dest_x, dest_y]] in cells and not (([x,y]  in position_cells)):
                cells.append([[x,y], [dest_x, dest_y]])
                blocked_cells[i] = [[x,y], [dest_x, dest_y]]
                position_cells.append([x,y])
                i-=1
            print (i)

        self.blocked_cells = blocked_cells

    def identify_anomaly(self,agent_x, agent_y, agent_dest_x, agent_dest_y, current_time):

        for cells in self.blocked_cells.values():
            myx, myy = self.E.getCellXY(agent_x, agent_y)
            #myx, myy = agent_x,agent_y
            myx_dest, myy_dest = self.E.getCellXY(agent_dest_x, agent_dest_y)
            #myx_dest, myy_dest = agent_dest_x,agent_dest_y
            if cells[0] == [myx, myy] and cells[1] == [myx_dest, myy_dest]:
                position = max(cells[0], cells[1])
                off= [+1.30123,+1.30123]
                if cells[0][0] == cells[1][0]:
                    off[0] = 0.5*20/5
                    off[1] = (cells[0][1] - cells[1][1]) * off[1]
                else:
                    off[0] = (cells[0][0] - cells[1][0]) *off[0]
                    off[1] = 0.5*20/5
                center = np.array([(position[0]-1) * 20 / 5.0, (position[1]-1) * 20 / 5.0]) + np.array(
                off)
                return [center[0],center[1]], "block"


        if current_time in self.flow_occurance:
            flow_effected_cells = self.flow_occurance[current_time]
            myx, myy = self.E.getCellXY(agent_x, agent_y)
            myx_dest, myy_dest = self.E.getCellXY(agent_dest_x, agent_dest_y)
            #myx, myy = agent_x,agent_y
            for cells in flow_effected_cells:
                if [myx, myy] == cells[0] :
                    position = cells[1]
                    off = [0.6*20/5 + 0.00123, 0.6*20/5 + 0.00123]
                    center = np.array([(position[0]-1) * 20 / 5.0, (position[1]-1) * 20 / 5.0]) + np.array(
                    off)
                    return [center[0],center[1]], "flow"

        return None, None

        #if self.time_anomaly == current_time:
        #    current_time = round(current_time + 0.3*current_time)
        #    return current_time




        #if [x,y] in self.blocked_cells.values():
        #    return "block"
        #if

if __name__ == "__main__":
    E = Grid([],x_range=20, y_range=20)
    anomaly = Anomalies(600, 0.5, 100, E)
    print (anomaly.identify_anomaly(2,1,3,1,0.0))
