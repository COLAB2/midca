import numpy as np
import time
import matplotlib.pyplot as plt
from gridworld import Grid
from AcousticTag import AcousticTag
from Agent import Agent
from AcousticReciever import AcousticReciever
import socket
import threading


def ComLink():
	global running, searchComplete, wp_list
	run=True
	while run:
		data = clientsocket.recv(1024)
		data = data.decode('utf-8')
		print(data)
		cmd=data.split(',')
		if cmd[0] == 'quit':
			running=False
			run=False
		if cmd[0] == 'start':
			running=True
		if cmd[0] == 'moveTo':
			x=int(cmd[1])-1
			y=int(cmd[2])-1
			center=np.array([x*200,y*200])+np.array([100, 100])
			wp_list[0]=[center]
		if cmd[0] == 'moveToPhysicalPosition':
			x=int(cmd[1])-1
			y=int(cmd[2])-1
			center=np.array([x,y])
			wp_list[0]=[center]
		if cmd[0] == 'inCell':
			agent=agentList[0]
			pos=agent.getPos()
			x=int(cmd[1])
			y=int(cmd[2])
			myx,myy=E.getCellXY(pos[0],pos[1])
			clientsocket.send(str.encode(str(x==myx and y==myy)))
		if cmd[0] == 'search':
			x=int(cmd[1])-1
			y=int(cmd[2])-1
			center=np.array([x*200,y*200])+np.array([100, 100])
			wp_list[0]=[center]
			wp_list[0]=search(wp_list[0], center)
			searchComplete=False
		
		if cmd[0] == 'searchComplete':
			clientsocket.send(str.encode(str(searchComplete)))
			
	
'''
psuedo code
create grid world
generate tag positons based on probability map and total number of fish N
create agents

simulation:
  1. detect tags
  2. update tag.lastPing
  3. updte agent dynamics

'''
def draw(x):
	plt.figure(1)
	plt.axis('scaled')
	plt.grid(True)
	plt.plot(x[0], x[1], 'r.')
	plt.xlim([0, 1000])
	plt.ylim([0, 1000])
	plt.xticks(np.arange(0,1000,200))
	plt.yticks(np.arange(0,1000,200))
    
	plt.draw()


def drawAgent(x):
	plt.figure(1)
	plt.axis('scaled')
	plt.grid(True)
	plt.plot(x[0], x[1], 'bo')
	plt.xlim([0, 1000])
	plt.ylim([0, 1000])
	plt.xticks(np.arange(0,1000,200))
	plt.yticks(np.arange(0,1000,200))
    
	plt.draw()
	
def iterative_average(x,n,ave):
	return ave+(x-ave)/(n+1)
	
def simulate_dynamics(agent,u,tspan,dt):
	inc = agent.state
	for i in np.linspace(tspan[0],tspan[1],int((tspan[1]-tspan[0])/dt)):
		inc+=agent.dynamics(inc,u)*dt#+world.flow(agent.getPos())*dt
	return inc
	
def f1_plan(x0, x, N):
	u = np.zeros(N)
	xsim = x0.copy()
	for n in range(N):
		e = x-xsim
		angle = np.arctan2(e[1], e[0])
		u[n] = angle
		xsim += 0.005*np.array([np.cos(u[n]), np.sin(u[n])])

	return u

def search(wp_list,X):
	#X is the center position of one of the 25 cells
	wp_list.append(np.array(X))
	wp_list.append(np.array(X)+np.array([-75,75]))
	wp_list.append(np.array(X)+np.array([75,75]))
	wp_list.append(np.array(X)+np.array([75,-75]))
	wp_list.append(np.array(X)+np.array([-75,-75]))
	wp_list.append(np.array(X))
	return wp_list
                   
def wp_track(x,wp_list):
	global  searchComplete
	e = np.array(wp_list[0])-x
	if np.linalg.norm(e) < 5 and len(wp_list) > 1:
		#print(wp_list)
		del wp_list[0]
                   
	if len(wp_list) == 1:
		searchComplete=True
	return wp_list, 1*np.arctan2(e[1],e[0])
                   
def m1_step(x,u):
        return 1*np.array([np.cos(u), np.sin(u)])
	
density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
			0.1, 0.3, 0.3, 0.1, 0.3,
			0.2, 0.3, 0.3, 0.2, 0.1,
			0.3, 0.9, 0.3, 0.2, 0.1,
			0.2, 0.3, 0.2, 0.1, 0.1])	

N = 1000 #how many tags present
simtime=100 #max simulation time
numAgents=1 #number of agents exploring
measurement_time = 1
time_step=.5

running=False
searchComplete=False
taglist=[]
agentList=[]
tagx=np.zeros(N)
tagy=np.zeros(N)
for i in range(N):
	#taglist.append(AcousticTag(i,last_ping=np.random.randn()),ping_delay=max(2,30*np.random.randn())) # most realistic 
	taglist.append(AcousticTag(i,last_ping=np.random.randn())) # more realistic (pings are not aligned in time)
	#taglist.append(AcousticTag(i)) #better for understanding because pings are aligned in time and  all have same ping interval
	x,y,_ = taglist[i].pos
	tagx[i]=x
	tagy[i]=y
	
E = Grid(taglist)
E.setMap(density_map)

for i in range(numAgents):
	s= AcousticReciever(np.array([0,0,0]),50)
	agentList.append(Agent(np.array([(i*2+1)*50.0,605.0]),s,E,dim=2))
	agentList[i].dynamics=m1_step
	
for i in range(N):
	x,y,_ = taglist[i].pos
	tagx[i]=x
	tagy[i]=y
	

#draw((tagx,tagy))
'''
for t in range(N):
    draw(taglist[t].pos)
   #plt.pause(.1)
''' 
# simulation
#input('Enter to begin simulation')


########################################################################
# create an INET, STREAMing socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# bind the socket to a public host, and a well-known port
sock.bind(('localhost', 80))
# become a server socket
sock.listen(5)
# accept connections from outside
(clientsocket, address) = sock.accept()
# now do something with the clientsocket
# in this case, we'll pretend this is a threaded server
x = threading.Thread(target=ComLink)
x.start()

##########################################################################
t=0
last_meas=t

# give some initial goals
wp_list = [[],[],[]]
wp_list[0] = search(wp_list[0], [100, 500])
wp_list[1] = search(wp_list[1], [900, 100])
wp_list[2] = search(wp_list[2], [900, 900])
while not running:
	pass
det_count=[0,0,0]
while t<=simtime or running: #change to better simulation stopping criteria 
	posx=np.zeros(numAgents)
	posy=np.zeros(numAgents)
	for i in range(len(agentList)):
		agent=agentList[i]
		pos=agent.getPos()
		#state = agent.getPos()#
		
		# compute input / update waypoint
		wp_list[i], u = wp_track(np.array(pos), wp_list[i])
		state=simulate_dynamics(agent,u, [0,time_step],.1)
		dets=agent.updateAgent(state,t)
		allDetectionData=agent.sensor.detectionList#history of every tag detection. includes (tag ID,time,agent pos,bin)

		posx[i]=pos[0]
		posy[i]=pos[1]
	for tag in taglist:
		tag.updatePing(t)
	t+=time_step
	plt.clf()
	draw((tagx,tagy))
	drawAgent((posx,posy))
	plt.pause(0.00001)#plt.pause(time_step)
	
	
	
for i in range(len(agentList)):
	agent=agentList[i]
	print("agent ",i," rate estimates")
	print(agent.belief_map)
	print("and measurements taken per cell")
	print(agent.belief_count)
	
print("True probability density  map")
print(E.p)
input('done')



