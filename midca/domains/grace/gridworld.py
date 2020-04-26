import numpy as np

''' Gridworld simulation:
    | 20 | 21 | 22 | 23 | 24 |
    | 15 | 16 | 17 | 18 | 19 |
    | 10 | 11 | 12 | 13 | 14 |
    | 5  | 6  | 7  | 8  | 9  |
    | 0  | 1  | 2  | 3  | 4  |

    - true position: x in [0, 5] x [0, 5] 
    - abstracted position: s in {0, 1, ..., 24}
    - true input: u in [-pi, pi]

    Grid centers:
    - 0: (0.5, 0.5)
    - 1: (1.5, 0.5)
    - 2: (2.5, 0.5)
    - 3: (3.5, 0.5)
    - 4: (4.5, 0.5)

    - 5: (0.5, 1.5)
    - 6: (1.5, 1.5)
    - 7: (2.5, 1.5)
    - 8: (3.5, 1.5)
    - 9: (4.5, 1.5)

    - 10: (0.5, 2.5)
    - 11: (1.5, 2.5)
    - 12: (2.5, 2.5)
    - 13: (3.5, 2.5)
    - 14: (4.5, 2.5)

    - 15: (0.5, 3.5)
    - 16: (1.5, 3.5)
    - 17: (2.5, 3.5)
    - 18: (3.5, 3.5)
    - 19: (4.5, .5)

    - 20: (0.5, 4.5)
    - 21: (1.5, 4.5)
    - 22: (2.5, 4.5)
    - 23: (3.5, 4.5)
    - 24: (4.5, 4.5)
'''

class Grid:
    def __init__(self):
        # starting mode
        self.m = 0

        # mode switch probability
        self.m_p = 0.0

        # starting true position
        self.x = np.array([0.5, 0.5])

        # starting abstracted position
        self.s = 0

        # fish detection probabilities
        self.p = np.array([0.1, 0.1, 0.4, 0.6, 0.2,
                           0.1, 0.3, 0.5, 0.8, 0.3,
                           0.2, 0.4, 0.2, 0.2, 0.1,
                           0.5, 0.9, 0.4, 0.2, 0.1,
                           0.3, 0.6, 0.3, 0.1, 0.1])

    def state(self):
        return (self.s, self.x)

    def act(self, u):
        bins = [0., 1., 2., 3., 4.]

        # simulate forward with inputs
        p = 0*u.copy()
        for t in range(len(u)):
            # f0 dynamics
            if self.m == 0:
                self.x += self.flow(self.x) + 0.005*np.array([np.cos(u[t]), np.sin(u[t])])

            # f1 dynamics
            elif self.m == 1:
                self.x += self.flow(self.x) + 0.005*np.array([-0.5*np.cos(u[t]), np.sin(u[t])])

            # randomly switch modes
            if np.random.rand() < self.m_p:
                self.m = 1

            # sample fish acoustics
            idx = np.digitize(self.x[0], bins)
            idy = np.digitize(self.x[1], bins)
            self.s = 5*(idy-1) + (idx-1)
            p[t] = np.random.rand() < self.p[self.s]

        return (self.s, self.x, p)

    def flow(self, x):
        return 0.004*np.array([np.cos(0.25*x[0]), np.sin(0.6*x[1])])

