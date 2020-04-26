import math

class Cell:
    def __init__(self, probability=0, mean=0):
        self.prob = probability
        self.mean = mean

    def __str__(self):
        return str(self.prob) + "," + str(self.mean)

class Grid:
    def __init__(self):
        self.grid = {}
        self.probabilities = []
        self.mean = []

    def calculate_grid(self, dim=5):
        for i in range(dim):
            for j in range(dim):
                self.grid["T"+str(i)+"x"+str(j)+"y"] = Cell()

    def initial_mean(self, dim=5):
        for i in range(dim):
            for j in range(dim):
                self.grid["T"+str(i)+"x"+str(j)+"y"].mean = i+j

    def calculate_prob(self, dim=5, x = 3):
        for i in range(dim):
            for j in range(dim):
                mean = self.grid["T"+str(i)+"x"+str(j)+"y"].mean
                self.grid["T"+str(i)+"x"+str(j)+"y"].prob = pow(mean, x) * math.exp(-1 * mean) / math.factorial(x)  # poisson distribution

    def display(self, dim=5):
        for i in range(dim):
            print ("")
            for j in range(dim):
                print (self.grid["T"+str(i)+"x"+str(j)+"y"]),

if __name__ == "__main__":
    g = Grid()
    g.calculate_grid()
    g.initial_mean()
    g.calculate_prob()
    g.display()