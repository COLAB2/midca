import numpy as np
import time
import matplotlib.pyplot as plt
from gridworld import Grid
import gridworld
from AcousticTag import AcousticTag
from Agent import Agent
from AcousticReciever import AcousticReciever
import socket
import threading


def ComLink():
	global u, run, updateGP,t
	run=True
	sock.send(str.encode(str(x_range)))
	while run:
		agent=agentList[0]
		st=agent.state
		bin=E.getAbstractPos(st[0],st[1])-1
		try:
			if not updateGP:
				sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(st[2])+" "+str(st[3])+" "+str(t)+" "+"None "))
			else:
				updateGP=False
				sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(st[2])+" "+str(st[3])+" "+str(t)+" "+str(latestMeas)))
		except:
			run=False
			t=simtime
		data = sock.recv(1024)
		data = data.decode('utf-8')
		cmd=data.split(',')
		if len(cmd)>1:
			u=(float(cmd[0]),float(cmd[1]))
			if st[0]>x_range or st[0]<0 or st[1]<0 or st[1]>y_range:
				_,utemp=wp_track(agent.getPos(),np.array([x_range/2,y_range/2]))
				u=np.clip(np.array([np.cos(utemp), np.sin(utemp)]),-0.1,0.1)
			#print(st,u)
			print(t,round(st[0],1),round(st[1],1),round(st[2],3),round(st[3],3),u)

	
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
	plt.xlim([0, E.x_range])
	plt.ylim([0, E.y_range])
	plt.xticks(np.arange(0,E.x_range,E.x_range/5.0))
	plt.yticks(np.arange(0,E.y_range,E.y_range/5.0))
    
	plt.draw()


def drawAgent(x,r=None):
	plt.figure(1)
	plt.axis('scaled')
	plt.grid(True)
	plt.plot(x[0], x[1], 'bo')
	if r==None:
		pass
	else:
		circ=plt.Circle((x[0], x[1]), r, color='b', fill=False)
		plt.gcf().gca().add_artist(circ)
	plt.xlim([0, E.x_range])
	plt.ylim([0, E.y_range])
	plt.xticks(np.arange(0,E.x_range,E.x_range/5.0))
	plt.yticks(np.arange(0,E.y_range,E.y_range/5.0))
    
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
	offset=.375*x_range
	wp_list.append(np.array(X))
	wp_list.append(np.array(X)+np.array([-offset,offset]))
	wp_list.append(np.array(X)+np.array([offset,offset]))
	wp_list.append(np.array(X)+np.array([offset,-offset]))
	wp_list.append(np.array(X)+np.array([-offset,-offset]))
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
                   
def m2_step(x,u):
	    #  |0 0   1    0|x        |0 0|
		#  |0 0   0    1|y     +  |0 0|u1
		#  |0 0  -a    0|vx       |1 0|u2
		#  |0 0   0   -b|vy       |0 1|
        a=0
        return np.matmul(1*np.array([[0, 0, 1, 0],[0,0,0,1],[0,0,-a,0],[0,0,0,-a]]),x)+np.matmul(1*np.array([[0,0],[0,0],[1,0],[0,1]]),u)

def m1_step(x,u):
        return u[0]*np.array([np.cos(u[1]), np.sin(u[1])])
		
density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
			0.1, 0.3, 0.3, 0.1, 0.3,
			0.2, 0.3, 0.3, 0.2, 0.1,
			0.3, 0.9, 0.3, 0.2, 0.1,
			0.2, 0.3, 0.2, 0.1, 0.1])	
# density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
			# 0.1, 0.9, 0.3, 0.1, 0.3,
			# 0.2, 0.3, 0.3, 0.2, 0.1,
			# 0.3, 0.3, 0.3, 0.2, 0.1,
			# 0.2, 0.3, 0.2, 0.1, 0.1])
# density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
			# 0.1, 0.3, 0.3, 0.1, 0.3,
			# 0.2, 0.3, 0.9, 0.2, 0.1,
			# 0.3, 0.1, 0.3, 0.2, 0.1,
			# 0.2, 0.3, 0.2, 0.1, 0.1])
N = 1000 #how many tags present
simtime=1500 #max simulation time
numAgents=1 #number of agents exploring
sensorRange=2
x_range=20.0
y_range=20.0
measurement_time = 2
time_step=.5
t=0
last_meas=t
run=False
running=False
searchComplete=False
updateGP=False
latestMeas=0
taglist=[]
agentList=[]
tagx=np.zeros(N)
tagy=np.zeros(N)
for i in range(N):
	#taglist.append(AcousticTag(i,last_ping=np.random.randn()),ping_delay=max(2,30*np.random.randn())) # most realistic 
	taglist.append(AcousticTag(i,last_ping=17*np.random.randn())) # more realistic (pings are not aligned in time)
	#taglist.append(AcousticTag(i)) #better for understanding because pings are aligned in time and  all have same ping interval
	x,y,_ = taglist[i].pos
	tagx[i]=x
	tagy[i]=y
	
E = Grid(taglist,x_range=x_range, y_range=y_range)
E.setMap(density_map)

for i in range(numAgents):
	s= AcousticReciever(np.array([0,0,0]),sensorRange)
	agentList.append(Agent(np.array([np.random.rand()*x_range,np.random.rand()*y_range,0,0]),s,E,dim=2))
	agentList[i].dynamics=m2_step
u=[0,0]	
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
# connect to ergodic controller
sock.connect(('localhost', 8080))
# now do something with the clientsocket
# in this case, we'll pretend this is a threaded server
x = threading.Thread(target=ComLink)
x.start()

##########################################################################



det_count=[0,0,0]
while t<=simtime:#or running: #change to better simulation stopping criteria 
	posx=np.zeros(numAgents)
	posy=np.zeros(numAgents)
	for i in range(len(agentList)):
		agent=agentList[i]
		pos=agent.getPos()
		#state = agent.getPos()#
		#srange=agent.sensor.range
		state=simulate_dynamics(agent,u, [0,time_step],.1)
		dets=agent.updateAgent(state,t)
		det_count[i]+=dets
		if last_meas+measurement_time<=t:
			updateGP = True
			bin=E.getAbstractPos(pos[0],pos[1])-1
			dtSet=agent.sensor.detectionSet
			rate_meas = len(dtSet)*1.0/measurement_time
			latestMeas=rate_meas
			agent.belief_count[bin]+=1
			agent.belief_map[bin]= iterative_average(rate_meas,agent.belief_count[bin],round(agent.belief_map[bin],3))  #iteratively average rate measurement
			if len(agent.sensor.detectionSet)>0:
				#print("agent ",i,", rate = ",rate_meas,",average rate = ",agent.belief_map[bin], " in bin ", bin)
				#print(last_meas,t,dtSet)
				agent.sensor.detectionSet=set()	
		posx[i]=pos[0]
		posy[i]=pos[1]
	if last_meas+measurement_time<=t:
		last_meas=t
	for tag in taglist:
		tag.updatePing(t)
	t+=time_step
	plt.clf()
	draw((tagx,tagy))
	drawAgent((posx,posy),r=sensorRange)
	plt.pause(0.00001)#plt.pause(time_step)
	
run=False
sock.send("end ".encode('utf-8'))
	
for i in range(len(agentList)):
	agent=agentList[i]
	print("agent ",i," rate estimates")
	agent.belief_map.shape=(5,5)
	print(np.flip(agent.belief_map,0))
	print("and measurements taken per cell")
	agent.belief_count.shape=(5,5)
	print(np.flip(agent.belief_count,0))
	
print("True probability density  map")
E.p.shape=(5,5)
print(np.flip(E.p,0))
#spacing=(50,50)
spacing=(1,1)
print("Rate field approximation for sensor with range",sensorRange," spaced at intervals of",spacing)
approx,pnts=E.approximateField(measurement_time,spacing=spacing,sensorRange=sensorRange,get_points=True)
#print(np.round(approx,decimals=2))
plt.figure(2)
plt.axis('scaled')
plt.grid(True)
#print('\n',pnts[:,:,0],'\n',pnts[:,:,1])
#plt.plot(pnts[:,:,0].flatten(), pnts[:,:,1].flatten(), 'r.',cmap='coolwarm')
plt.contourf(pnts[:,:,0], pnts[:,:,1], np.flip(np.round(approx,decimals=2),(0,1)).transpose(), 20, cmap='coolwarm')# cmap='inferno'), cmap='RdGy')


cbar = plt.colorbar()
cbar.set_label('Detection rate')
plt.xlim([0, x_range])
plt.ylim([0, y_range])
plt.xticks(np.arange(0,x_range,spacing[0]))
plt.yticks(np.arange(0,y_range,spacing[1]))
plt.draw()
plt.pause(0.00001)
input('done')



