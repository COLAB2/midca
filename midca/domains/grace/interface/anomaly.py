import random
from .gridworld import Grid
import numpy as np
import operator
import time

class Anomalies:

    def __init__(self, simtime, time_step, seed, E, publisher, draw=False):
        self.publisher = publisher
        self.simtime = simtime
        self.row_dimension = 5
        self.col_dimension = 5
        self.time_step = time_step
        self.E = E
        random.seed(seed)
        self.flow_occurance = None
        self.blocked_cells = None
        self.time_anomaly = None
        self.draw = draw
        #self.flow_anomaly(60)
        self.block_anomaly(15, 8)




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

    def block_anomaly(self, number_block_cells, temporary_blocks):
        blocked_cells = {}
        cells = []
        position_cells = []
        rows = [i for i in range(0, self.row_dimension)]
        cols = [i for i in range(0, self.col_dimension)]
        i = number_block_cells
        while(i):
            x = random.choice(rows)
            y = random.choice(cols)
            dest_x , dest_y = self.throw_direction(x,y)
            if not [[x,y], [dest_x, dest_y]] in cells and not (([x,y]  in position_cells))\
                    and not [[dest_x,dest_y], [x, y]] in cells:
                cells.append([[x,y], [dest_x, dest_y]])
                blocked_cells[i] = [[x,y], [dest_x, dest_y]]
                position_cells.append([x,y])
                i-=1
            #print (i)

        self.blocked_cells = blocked_cells
        self.temporary_blocks = random.sample(list(range(1,number_block_cells)), temporary_blocks-1)
        if self.draw:
            self.draw_markers()

    def allow_temporary_blocks(self, initial_position, dest_position):
        agent_x, agent_y = initial_position[0], initial_position[1]
        agent_dest_x, agent_dest_y = dest_position[0], dest_position[1]
        if self.blocked_cells:
            for key,cells in list(self.blocked_cells.items()):
                myx, myy = agent_x, agent_y#self.E.grid_to_sim(agent_x, agent_y)
                myx_dest, myy_dest = agent_dest_x, agent_dest_y #self.E.grid_to_sim(agent_dest_x, agent_dest_y)
                if (cells[0] == [myx, myy] and cells[1] == [myx_dest, myy_dest]) :
                  if key in self.temporary_blocks:
                    del self.blocked_cells[key]
                  return True
                elif (cells[1] == [myx, myy] and cells[0] == [myx_dest, myy_dest]):
                  if key in self.temporary_blocks:
                    del self.blocked_cells[key]
                  return True

    def isPartialblock(self, initial_position, dest_position):
        agent_x, agent_y = initial_position[0], initial_position[1]
        agent_dest_x, agent_dest_y = dest_position[0], dest_position[1]
        if self.blocked_cells:
            for key,cells in list(self.blocked_cells.items()):
                myx, myy = agent_x, agent_y#self.E.grid_to_sim(agent_x, agent_y)
                myx_dest, myy_dest = agent_dest_x, agent_dest_y #self.E.grid_to_sim(agent_dest_x, agent_dest_y)
                if (cells[0] == [myx, myy] and cells[1] == [myx_dest, myy_dest]) :
                  return False
                elif (cells[1] == [myx, myy] and cells[0] == [myx_dest, myy_dest]):
                  return False
            return True

    def draw_markers(self):

        def send_markers(publisher, position, index, direction, color="green"):
            positions = []
            if direction[0]:
                positions.append(position)

                position1 = [position[0], position[1] - 20]
                positions.append(position1)

                position1 = [position[0], position[1] + 20]
                positions.append(position1)

                position1 = [position[0], position[1] - 40]
                positions.append(position1)

                position1 = [position[0], position[1] + 40]
                positions.append(position1)

            else:
                positions.append(position)

                position1 = [position[0]-20, position[1]]
                positions.append(position1)

                position1 = [position[0]+20, position[1]]
                positions.append(position1)

                position1 = [position[0]-40, position[1]]
                positions.append(position1)

                position1 = [position[0]+40, position[1]]
                positions.append(position1)

            for i, position in enumerate(positions):

                message = [b"AddMarker",
                           f"type=gateway,x={position[0]},y={position[1]},scale=4.3,label=block{index}{i},color={color},width=4.5".encode()]
                time.sleep(0.1)
                self.publisher.send_multipart(message)



        for index,blockade in list(self.blocked_cells.items()):
            #print (blockade)
            x,y = self.E.grid_to_sim(blockade[0][0], blockade[0][1])
            x1, y1 = self.E.grid_to_sim(blockade[1][0], blockade[1][1])
            position = (x+x1)/2 , (y+y1)/2
            direction = self.get_threshold(blockade)
            color = "red"
            if index in self.temporary_blocks:
                color = "green"
            send_markers(self.publisher, position, index, direction, color)



    def subract(self, e, e1, threshold=10):
        result = e - e1
        if result == 0:
            return result
        if result > 0:
            result = result - threshold
        else:
            result =result + threshold
        return result


    def get_threshold(self, cells):
        threshold = list(map(self.subract, cells[0], cells[1]))
        return threshold


    def identify_anomaly(self,agent_x, agent_y, agent_dest_x, agent_dest_y, current_time):

        if self.blocked_cells:
            for cells in list(self.blocked_cells.values()):
                myx, myy = agent_x, agent_y#self.E.grid_to_sim(agent_x, agent_y)
                myx_dest, myy_dest = agent_dest_x, agent_dest_y #self.E.grid_to_sim(agent_dest_x, agent_dest_y)
                if (cells[0] == [myx, myy] and cells[1] == [myx_dest, myy_dest]) :
                    x,y = self.E.grid_to_sim(agent_x, agent_y)
                    x1, y1 = self.E.grid_to_sim(agent_dest_x, agent_dest_y)
                    print((x,y))
                    print((x1,y1))
                    position = (x+x1)/2 , (y+y1)/2
                    threshold = self.get_threshold(cells)
                    position = list(map(operator.sub, position, threshold))
                    #position = list(set(position) - set(threshold))
                    return position, "block"
                elif (cells[1] == [myx, myy] and cells[0] == [myx_dest, myy_dest]):
                    x,y = self.E.grid_to_sim(agent_dest_x, agent_dest_y)
                    x1, y1 = self.E.grid_to_sim(agent_x, agent_y)
                    print((x,y))
                    print((x1,y1))
                    position = (x+x1)/2 , (y+y1)/2
                    threshold = [each * -1 for each in  self.get_threshold(cells)]
                    position = list(map(operator.sub, position, threshold))
                    #position = list(set(position) - set(threshold))
                    return position, "block"

        """
        if current_time in self.flow_occurance:
            flow_effected_cells = self.flow_occurance[current_time]
            myx, myy = self.E.grid_to_sim(agent_x, agent_y)
            myx_dest, myy_dest = self.E.grid_to_sim(agent_dest_x, agent_dest_y)
            #myx, myy = agent_x,agent_y
            for cells in flow_effected_cells:
                if [myx, myy] == cells[0] :
                    position = cells[1]
                    off = [0.6*20/5 + 0.00123, 0.6*20/5 + 0.00123]
                    center = np.array([(position[0]-1) * 20 / 5.0, (position[1]-1) * 20 / 5.0]) + np.array(
                    off)
                    return [center[0],center[1]], "flow"
        """
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
    print((anomaly.identify_anomaly(2,1,3,1,0.0)))
