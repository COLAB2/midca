import socket
import time

class TagWorld():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 5700))
        self.sock.settimeout(1)

    def runSim(self):
        self.sock.send(str.encode(str('start')))

    def endSim(self):
        self.sock.send(str.encode(str('quit')))

    def getAdjacent(self, position):
        msg = "get_tags_adjacent," + str(position[0]+1) + "," + str(position[1]+1)
        self.sock.send(str.encode(str(msg)))
        msg = self.sock.recv(2048)
        data = msg.decode('utf-8')
        data = data.split(",")
        data = [float(each) for each in data]
        return data

    def UpdateUncertainity(self, mode, radius):
        msg = "updateUncertainity," + str(mode) + "," + str(radius)
        self.sock.send(str.encode(str(msg)))

    def UpdateSurfstatus(self, mode):
        msg = "ascendordescend," + str(mode)
        self.sock.send(str.encode(str(msg)))


    # initial_position and final_position is a grid cell of format [x,y]
    # template to move from one cell to another
    def move_cell(self, initial_position, final_destination):
        msg = "moveTo," + str(final_destination[0]+1) + "," + str(final_destination[1]+1)
        self.sock.send(str.encode(str(msg)))
        # make the movement from initial_position to the final destination

        # return acknowledgment once it moved to the destination grid cell
        return True  # boolean value; true if reached

    # position = [x, y]  # x and y are the coordinates of the grid cell
    def in_cell(self, position):
        msg = "inCell," + str(position[0]) + "," + str(position[1])
        self.sock.send(str.encode(str(msg)))
        msg = self.sock.recv(2048)
        return msg.decode('utf-8')

    def get_cell(self):
        try:
            msg = "getCell"
            self.sock.send(str.encode(str(msg)))
            msg = self.sock.recv(2048)
            position = msg.decode('utf-8')
            # convert to midca coordinates
            # for example [2,2] is [1,1] in our representation
            if position:
                position = position.split(",")
                position = [str(int(pos)-1) for pos in position]
                position = ",".join(position)
            return position
        except Exception as e:
            print e

    # position = [x, y]  # x and y are the coordinates of the grid cell
    def search_and_get_tags2(self, position):
        # search the location (grid cell [x,y])
        if self.searchComplete() == 'True':
            return self.get_tags()  # no_of_tags
        else:
            self.search(position)

    def search_and_get_tags(self, position):
        # search the location (grid cell [x,y])
        self.search(position)
        complete = 'False'
        while complete == 'False':
            complete = self.searchComplete()
        # collect tags
        print(complete)
        return self.get_tags()  # no_of_tags

    # position = [x, y]  # x and y are the coordinates of the grid cell

    def search(self, position):
        msg = "search," + str(position[0]+1) + "," + str(position[1]+1)
        self.sock.send(str.encode(str(msg)))


    def send(self, position):
        msg = str(position[0]+1) + "," + str(position[1]+1)
        self.sock.send(str.encode(str(msg)))

    # search the location (grid cell [x,y])
    # display the movement of the agent in the cell

    def searchComplete(self):
        self.sock.send(str.encode('searchComplete'))
        msg = self.sock.recv(2048)
        return msg.decode('utf-8')

    def get_tags(self, position):
        try:
            msg = "get_tags," + str(position[0]+1) + "," + str(position[1]+1)
            self.sock.send(str.encode(str(msg)))
            # collect tags
            msg = self.sock.recv(2048)
            return int(msg.decode('utf-8'))
        except:
            return False

    def get_measurement(self, position=None):
        try:
            msg = "get_measurement"
            self.sock.send(str.encode(str(msg)))
            # collect tags
            msg = self.sock.recv(2048)
            return msg.decode('utf-8')
        except:
            return False

    def get_cell_poisson_rate(self, pos=None):
        msg = "cell_lambda"
        if type(pos) != type(None):
            msg += "," + str(pos[0]+1) + "," + str(pos[1]+1)
        self.sock.send(str.encode(str(msg)))
        # collect tags
        msg = self.sock.recv(1024)
        return float(msg.decode('utf-8'))

    def simtime(self):
        try:
            msg = "time"
            self.sock.send(str.encode(str(msg)))
            msg = self.sock.recv(2048)
            time = msg.decode('utf-8')
            return float(time)
        except Exception as e:
            print e

    def close(self):
        self.sock.shutdown(1)
        self.sock.close()


if __name__ == "__main__":
        """"
        tag = TagWorld()
        tag.runSim()


        tag = TagWorld()
        tag.move_cell([0,0], [1,3])

        """


        tag = TagWorld()
        tag.runSim()

        tag = TagWorld()
        tag.search([0,1])

        tag = TagWorld()
        print (tag.get_measurement())

        #print (tag.simtime())
        #tag.search([0, 3])
        #print tag.get_cell_poisson_rate()
        #print tag.get_tags()
        #tag.send([3,3])
        #print (msg)
        #tag.UpdateUncertainity("High", 100)
        #tag.UpdateUncertainity("Med", 50)
        #tag.UpdateSurfstatus("ascend")




