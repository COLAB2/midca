from Agent import Agent
from AcousticReciever import AcousticReciever
from gridworld import Grid
import numpy as np

def m2_step(x,u): #motion model
	    #  |0 0   1    0|x        |0 0|
		#  |0 0   0    1|y     +  |0 0|u1
		#  |0 0  -a    0|vx       |1 0|u2
		#  |0 0   0   -b|vy       |0 1|
        a=.25
        return np.matmul(1*np.array([[0, 0, 1, 0],[0,0,0,1],[0,0,-a,0],[0,0,0,-a]]),x)+np.matmul(1*np.array([[0,0],[0,0],[1,0],[0,1]]),u)
		
#an agent needs a sensor object
#the sensor needs a position and range
sensorRange=5
sensor= AcousticReciever(np.array([0,0,0]),sensorRange)
#it also needs a gridworld object for access to the tag locations and possibly flow model. may get rid of the equirment in future
taglist=[]
E = Grid(taglist,x_range=100, y_range=100)
#agent takes initial state array, a sensor, and the number of dimensions (2 for (x,y) vs 3 for (x,y,z)) can also be added 
rd2d=Agent(np.array([1,1,0,0]),sensor,E,dim=2)
rd2d.dynamics=m2_step #then agent takes a function as the dynamics with the input signature f(x,u)