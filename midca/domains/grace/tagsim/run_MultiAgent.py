import numpy as np
import time
import matplotlib.pyplot as plt
from gridworld import Grid
from AcousticTag import AcousticTag
from Agent import Agent
from AcousticReciever import AcousticReciever
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
	e = np.array(wp_list[0])-x
	if np.linalg.norm(e) < 5 and len(wp_list) > 1:
		print(wp_list)
		del wp_list[0]
                   
	return wp_list, 1*np.arctan2(e[1],e[0])
                   
def m1_step(x,u):
        return 1*np.array([np.cos(u), np.sin(u)])
	
density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
			0.1, 0.3, 0.3, 0.1, 0.3,
			0.2, 0.3, 0.3, 0.2, 0.1,
			0.3, 0.9, 0.3, 0.2, 0.1,
			0.2, 0.3, 0.2, 0.1, 0.1])	

N = 1000 #how many tags present
simtime=10 #max simulation time
numAgents=3 #number of agents exploring
measurement_time = 1
time_step=.5

taglist=[]
agentList=[]
tagx=np.zeros(N)
tagy=np.zeros(N)
for i in range(N):
	#taglist.append(AcousticTag(i,last_ping=30*np.random.randn()),ping_delay=max(2,30*np.random.randn())) # most realistic 
	taglist.append(AcousticTag(i,last_ping=15*np.random.randn())) # more realistic (pings are not aligned in time)
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
t=0
last_meas=t

# give some initial goals
wp_list = [[],[],[]]
wp_list[0] = search(wp_list[0], [100, 500])
wp_list[1] = search(wp_list[1], [900, 100])
wp_list[2] = search(wp_list[2], [900, 900])
while t<=simtime: #change to better simulation stopping criteria 
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
		if dets>0:
			pass#print(dets,' detections',t,state)
		if last_meas+measurement_time<=t:
			bin=E.getAbstractPos(pos[0],pos[1])-1
			dtSet=agent.sensor.detectionSet
			rate_meas = len(dtSet)*1.0/measurement_time
			agent.belief_count[bin]+=1
			agent.belief_map[bin]= iterative_average(rate_meas,agent.belief_count[bin],agent.belief_map[bin])  #iteratively average rate measurement
			if len(agent.sensor.detectionSet)>0:
				print("agent ",i,", rate = ",rate_meas,",average rate = ",agent.belief_map[bin], " in bin ", bin)
				print(last_meas,t,dtSet)
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
	drawAgent((posx,posy))
	plt.pause(0.00001)#plt.pause(time_step)
	
	
	
for i in range(len(agentList)):
	agent=agentList[i]
	print("agent ",i," rate estimates")
	agent.belief_map.shape=(5,5)
	print(np.flip(agent.belief_map,0))
	print("and measurements taken per cell")
	agent.belief_count.shape=(5,5)
	print(np.flip(agent.belief_count,0))
	

print("rate map for lambda with tau = ",measurement_time)
print(np.round(E.groundTruth(measurement_time),decimals=2))
print("True probability density  map")
E.p.shape=(5,5)
print(np.flip(E.p,0))
spacing=(25,25)
sensorRange=50
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
plt.xlim([0, 1000])
plt.ylim([0, 1000])
plt.xticks(np.arange(0,1000,max(spacing[0],50)))
plt.yticks(np.arange(0,1000,max(spacing[1],50)))
plt.draw()
plt.pause(0.00001)
input('done')



