import numpy as np
import matplotlib.pyplot as plt
from gridworld import Grid

def draw(s, x):
    plt.figure(1)
    plt.axis('scaled')
    plt.grid(True)
    plt.plot(x[0], x[1], 'ro')
    plt.xlim([0, 5])
    plt.ylim([0, 5])
    plt.xticks(np.arange(0,6,1))
    plt.yticks(np.arange(0,6,1))
    
    plt.draw()
    plt.pause(0.0001)


def f1_plan(x0, x, N):
    u = np.zeros(N)
    xsim = x0.copy()
    for n in range(N):
        e = x-xsim
        angle = np.arctan2(e[1], e[0])
        u[n] = angle
        xsim += 0.005*np.array([np.cos(u[n]), np.sin(u[n])])

    return u

def run():
    E = Grid()

    #s is 0
    #x is (0.5, 0.5)
    s, x = E.state()

    # final destination
    g = np.array([0.8, 0.8])

    # 20 steps
    for t in range(10):
        # plan inputs without a flow model
        u = f1_plan(x, g, 50)


        # get next surfacing
        # p = rand(0,1) < self.exp_probability(s)
        s, x, p = E.act(u)

        print (s)
        print(x)
        print (p)

        # show average acoustic over trajectory
        print((s, tuple(x), np.mean(p)))
        draw(s, x)
        plt.pause(1)


#global dictionary
# structure of grid_tags = {"xy" :[ [0,0,....1],  [0,0,....1] ..., [0,0,....1]]
grid_tags = {}
index = 0


class Tags:

    def __init__(self):
        self.grid_tags = {}
        self.index = 0

    def generate_tags(self, position):
        """
        :param position: list in the format [x,y]
        :return: update grid_tags
        """
        E = Grid()
        # initial destination
        E.x = np.array([position[0] + 0.2, position[1] + 0.2])
        s, x = E.state()

        # final destination
        g = np.array([position[0] + 0.8, position[1] + 0.8])

        # initialize position in grid_tags
        xy = "".join(str(e) for e in position)
        self.grid_tags[xy] = []

        for t in range(10):
            # plan inputs without a flow model
            u = f1_plan(x, g, 50)

            # get next surfacing
            # p = rand(0,1) < self.exp_probability(s)
            s, x, p = E.act(u)

            # append it to grid_tags
            self.grid_tags[xy].append(p)

            draw(s, x)
            plt.pause(1)
        plt.pause(0.1)

    def get_tags(self, position):
        """
        :param position: list in the format [x,y]
        :return: probability values a list of 10
        """

        # check if the position is in the global variable
        xy = "".join(str(e) for e in position)
        if xy in self.grid_tags:
            values = self.grid_tags[xy][self.index].copy()
            self.index +=1
            # delete the position from dictionary
            if self.index == 10:
                del self.grid_tags[xy]
            return values

        # else calculate the values and add it to the dictionary and send first value
        else:
            self.generate_tags(position)
            return self.get_tags(position)


if __name__ == "__main__":
    run()