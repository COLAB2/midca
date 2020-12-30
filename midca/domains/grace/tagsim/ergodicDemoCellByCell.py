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
		  
def doubleIntegratorErgodicControl(agent,update,scale=None,offsets=None):
	global run,t
	st=agent.state
	if scale!=None:
		sock.send(str.encode("change_scale "+str(scale)+"\n "))
		read
	else:
		scale=x_range
	if offsets!=None:
		sock.send(str.encode("offset "+str(offsets[0])+" "+str(offsets[1])+"\n "))
	else:
		offsets=(0,0)
	try:
		if not update:
			sock.send(str.encode(str((round(st[0],1)-offsets[0])/scale)+" "+str((round(st[1],1)-offsets[1])/scale)+" "+str(st[2])+" "+str(st[3])+" "+str(t)+" "+"None "))
		else:
			sock.send(str.encode(str((round(st[0],1)-offsets[0])/scale)+" "+str((round(st[1],1)-offsets[1])/scale)+" "+str(st[2])+" "+str(st[3])+" "+str(t)+" "+str(latestMeas)))
	except:
		run=False
		t=simtime
		return
	data = sock.recv(1024)
	data = data.decode('utf-8')
	cmd=data.split(',')
	if len(cmd)>1:
		u=(float(cmd[0]),float(cmd[1]))
		if st[0]>offsets[0]+scale or st[0]<offsets[0] or st[1]<offsets[1] or st[1]>offsets[1]+scale:
			_,utemp=wp_track(agent.getPos(),np.array([(offsets[0]+scale)/2,(offsets[1]+scale)/2]))
			u=np.clip(np.array([np.cos(utemp), np.sin(utemp)]),-0.5,0.5)
		return u

def singleIntegratorErgodicControl(agent,update,scale=None,offsets=None):
	global run,t,current_scale,current_offsets
	st=agent.state
	if scale!=None:
		if current_scale != scale:
			sock.send(str.encode("change_scale "+str(scale)))
			confirm=sock.recv(1024)
			current_scale = scale if "confirm" in confirm.decode('utf-8') else x_range
	else:
		scale=x_range
	if offsets!=None:
		if current_offsets != offsets:
			sock.send(str.encode("offset "+str(offsets[0])+" "+str(offsets[1])+"\0"))
			confirm=sock.recv(1024)
			current_offsets = offsets if "confirm" in confirm.decode('utf-8') else (0,0)
	else:
		offsets=(0,0)
	try:
		if not update:
			sock.send(str.encode(str((round(st[0],1)-offsets[0])/scale)+" "+str((round(st[1],1)-offsets[1])/scale)+" "+str(t)+" "+"None "))
		else:
			sock.send(str.encode(str((round(st[0],1)-offsets[0])/scale)+" "+str((round(st[1],1)-offsets[1])/scale)+" "+str(t)+" "+str(latestMeas)))
	except:
		run=False
		t=simtime
		return	
	data = sock.recv(1024)
	data = data.decode('utf-8')
	cmd=data.split(',')
	print(data)
	if len(cmd)>1:
		u=(float(cmd[0]),float(cmd[1]))
		if st[0]>offsets[0]+scale or st[0]<offsets[0] or st[1]<offsets[1] or st[1]>offsets[1]+scale:
			_,utemp=wp_track(agent.getPos(),np.array([[offsets[0]+scale/2.0,offsets[1]+scale/2.0]]))
			u=np.clip(np.array([np.cos(utemp), np.sin(utemp)]),-1,1)
		return u
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
	
	
def wp_track(x,wp_list):
	global  searchComplete
	e = np.array(wp_list[0])-x
	if np.linalg.norm(e) < x_range/50.0 and len(wp_list) > 1:
		print("waypoint list",np.linalg.norm(e) < x_range/50.0,len(wp_list) > 1,wp_list)
		del wp_list[0]
                   
	if len(wp_list) == 1:
		searchComplete=True
	return wp_list, 1*np.arctan2(e[1],e[0])
	
########################   motion models  ###################################                  
def m1_step(x,u):
        return 1*np.array([np.cos(u), np.sin(u)])
		
def m2_step(x,u):
	    #  |0 0   1    0|x        |0 0|
		#  |0 0   0    1|y     +  |0 0|u1
		#  |0 0  -a    0|vx       |1 0|u2
		#  |0 0   0   -b|vy       |0 1|
        a=.25
        return np.matmul(1*np.array([[0, 0, 1, 0],[0,0,0,1],[0,0,-a,0],[0,0,0,-a]]),x)+np.matmul(1*np.array([[0,0],[0,0],[1,0],[0,1]]),u)

def m3_step(x,u):
        return np.array([max(min(u[0],1),-1),max(min(u[1],1),-1)])

############################# test functions  ###############################################
def rastrigin(x,y):
	return 20+x**2+y**2-10*(np.cos(2*np.pi*x)+np.cos(2*np.pi*y))

def rosenbrock(x,y):
	a,b=(10,.001)
	return b*(y-x**2)**2+(a-x)**2
	
def gaussianSum(x,y):
	r1 = np.array([.75*x_range,.45*y_range])
	r2 = np.array([.3*x_range,.7*y_range])
	loc = np.array([x,y])
	return 10*np.exp(-0.05*np.linalg.norm(loc-r1)**2)+15*np.exp(-0.1*np.linalg.norm(loc-r2)**2)

def tagField(tagData,pos,t,time_step,sensorRange):
	#last_ping=tagData[:,0],posx=tagData[:,1],posy=tagData[:,2],posz=tagData[:,3],delay=tagData[:,4],ID=tagData[:,5],bin=tagData[:,6]=tagData
	#diff=tagData[:,1:3]-np.array([pos[0],pos[1]]) 
	distance=np.linalg.norm(tagData[:,1:3]-np.array([pos[0],pos[1]]),axis=1)
	eps=time_step/100.0
	c1=(np.fmod(t,tagData[:,4]+eps)-(tagData[:,0]+tagData[:,4]))<time_step
	c2=(np.fmod(t,tagData[:,4]+eps)>(tagData[:,0]+tagData[:,4]))
	pinging = np.logical_and(c1,c2)
	dtSet= np.logical_and(distance<sensorRange,pinging)
	return tagData[np.where(pinging)[0],:],tagData[np.where(dtSet)[0],5],np.sum(dtSet)#pinging,detection set,detectionNum
	
density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
			0.1, 0.3, 0.3, 0.1, 0.3,
			0.2, 0.3, 0.3, 0.2, 0.1,
			0.3, 0.9, 0.3, 0.2, 0.1,
			0.2, 0.3, 0.2, 0.1, 0.1])	
#################################### simulation settings   ###################################
N = 1000 #how many tags present
simtime=1000 #max simulation time
numAgents=1 #number of agents exploring
sensorRange=2
x_range=20.0 #grid size
y_range=20.0
spacing=(1,1)#(.5,.5) #spacing between points for visualizing fields
searchMethods = ["ERGODIC_DI","ERGODIC_SI"]
method = searchMethods[1]
fields= ["tag","gassian sum","rosenbrock","rastrigin"]
fieldMax = [(5.5,14,7),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5
field = fields[0]
measurement_time = 2.0
time_step=.5
start_pos=(.05*x_range,.1*y_range)#
show_only_when_pinging=True
stopOnMax = False
visualize = True
logData=False
###############################################################################################

current_scale=x_range
current_offsets=(0,0)
t=0
last_meas=t
run=False
running=False
searchComplete=False
updateGP=False

latestMeas=0
u=0

taglist=[]
agentList=[]
tagx=np.zeros(N)
tagy=np.zeros(N)
#for i in range(N):
	#taglist.append(AcousticTag(i,last_ping=-np.random.rand()),ping_delay=max(2,30*np.random.randn())) # most realistic 
#	taglist.append(AcousticTag(i,last_ping=-17*np.random.rand())) # more realistic (pings are not aligned in time)
	#taglist.append(AcousticTag(i)) #better for understanding because pings are aligned in time and  all have same ping interval
#	x,y,_ = taglist[i].pos
#	tagx[i]=x
#	tagy[i]=y
	
E = Grid(taglist,x_range=x_range, y_range=y_range)
if field == fields[0]:
	taglist=E.loadTagList()#E.setMap(density_map)
	tagData=np.genfromtxt("tags.csv",delimiter=",")
	#E.saveTagList()
for i in range(numAgents):
	s= AcousticReciever(np.array([0,0,0]),sensorRange)
	if method == searchMethods[0]:
		#agentList.append(Agent(np.array([np.random.rand()*x_range,np.random.rand()*y_range,0,0]),s,E,dim=2))
		agentList.append(Agent(np.array([start_pos[0],start_pos[1],0,0]),s,E,dim=2))
		agentList[i].dynamics=m2_step
		u=[0,0]	
	elif method == searchMethods[1]:
		#agentList.append(Agent(np.array([np.random.rand()*x_range,np.random.rand()*y_range,0,0]),s,E,dim=2))
		agentList.append(Agent(np.array([start_pos[0],start_pos[1]]),s,E,dim=2))
		agentList[i].dynamics=m3_step
		u=[0,0]	
	else:
		#agentList.append(Agent(np.array([np.random.rand()*x_range,np.random.rand()*y_range]),s,E,dim=2))
		agentList.append(Agent(np.array([start_pos[0],start_pos[1]]),s,E,dim=2))
		agentList[i].dynamics=m1_step
		
for i in range(len(taglist)):
	x,y,_ = taglist[i].pos
	tagx[i]=x
	tagy[i]=y
		
if field == fields[1]:
	nx_bins = int(x_range/spacing[0])
	ny_bins = int(y_range/spacing[1])
	x_bins=np.array(range(nx_bins))*x_range/nx_bins
	y_bins=np.array(range(ny_bins))*y_range/ny_bins
	plottingPoints = [(idx+spacing[0]/2.0,idy+spacing[1]/2.0,gaussianSum(idx+spacing[0]/2.0,idy+spacing[1]/2.0)) for idx in x_bins for idy in y_bins]
	plottingPoints =  np.array(plottingPoints)
	plottingPoints.shape=(nx_bins,ny_bins,3)
if field == fields[2]:
	nx_bins = int(x_range/spacing[0])
	ny_bins = int(y_range/spacing[1])
	x_bins=np.array(range(nx_bins))*x_range/nx_bins
	y_bins=np.array(range(ny_bins))*y_range/ny_bins
	plottingPoints = [(idx+spacing[0]/2.0,idy+spacing[1]/2.0,rosenbrock(idx+spacing[0]/2.0,idy+spacing[1]/2.0)) for idx in x_bins for idy in y_bins]
	plottingPoints =  np.array(plottingPoints)
	plottingPoints.shape=(nx_bins,ny_bins,3)
if field == fields[3]:
	nx_bins = int(x_range/spacing[0])
	ny_bins = int(y_range/spacing[1])
	x_bins=np.array(range(nx_bins))*x_range/nx_bins
	y_bins=np.array(range(ny_bins))*y_range/ny_bins
	plottingPoints = [(idx+spacing[0]/2.0,idy+spacing[1]/2.0,rastrigin(idx+spacing[0]/2.0,idy+spacing[1]/2.0)) for idx in x_bins for idy in y_bins]
	plottingPoints =  np.array(plottingPoints)
	plottingPoints.shape=(nx_bins,ny_bins,3)
 
	

#draw((tagx,tagy))
'''
for t in range(N):
    draw(taglist[t].pos)
   #plt.pause(.1)
''' 
# simulation
#input('Enter to begin simulation')


#########################  socket threads ###############################################
# create an INET, STREAMing socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
if method == searchMethods[0] or method == searchMethods[1]:
	# connect to ergodic controller
	sock.connect(('localhost', 8080))
	sock.send(str.encode(str(x_range)))	

det_count=[0,0,0]
################################################ simulation loop ####################################
endSim=False
maxMeas=0
sc=current_scale
while t<=simtime:#or running: #change to better simulation stopping criteria 
	posx=np.zeros(numAgents)
	posy=np.zeros(numAgents)
	pinging_x =np.zeros(1)
	pinging_y =np.zeros(1)
	#print(t)
	for i in range(len(agentList)):
		agent=agentList[i]
		pos=agent.getPos()
		#state = agent.getPos()#
		if method == searchMethods[0]:
			u=doubleIntegratorErgodicControl(agent,updateGP,scale=x_range/5.0,offsets=(1*x_range/5.0,2*x_range/5.0))
			if updateGP:
				updateGP=False
		if method == searchMethods[1]:
			if t%50==0:
				off=(round(np.random.rand()*4)*x_range/5.0,round(np.random.rand()*4)*x_range/5.0)
				sc=x_range/5.0
			u=singleIntegratorErgodicControl(agent,updateGP,scale=sc,offsets=off)
			print(u)
			if updateGP:
				updateGP=False
		state=simulate_dynamics(agent,u, [0,time_step],.1)
		dets=agent.updateAgent(state,t)
		pos=agent.getPos()
		if field == fields[0]:
			pinging,detSet,dets2=tagField(tagData,pos,t,time_step,sensorRange)
			#print(t,pinging.shape,dets,dets,detSet,agent.sensor.detectionSet)
			allDetectionData = agent.sensor.detectionList  # history of every tag detection. includes (tag ID,time,agent pos,bin)
			det_count[i]+=dets
		if field == fields[3]:
			latestMeas=rastrigin(pos[0],pos[1])
			if last_meas+measurement_time<=t:
				updateGP = True
		elif field == fields[2]:
			latestMeas=rosenbrock(pos[0],pos[1])
			if last_meas+measurement_time<=t:
				updateGP = True
		elif field == fields[1]:
			latestMeas=gaussianSum(pos[0],pos[1])
			bin = E.getAbstractPos(pos[0], pos[1]) - 1
			allMeasurementData.append([latestMeas, t, [pos[0], pos[1]], bin])															 
			if latestMeas >= fieldMax[1][2]:
				endSim=True
			if last_meas+measurement_time<=t:
				updateGP = True
		elif field == fields[0]:
			if last_meas+measurement_time<=t:
				updateGP = True
				bin=E.getAbstractPos(pos[0],pos[1])-1
				dtSet=agent.sensor.detectionSet
				rate_meas = len(dtSet)*1.0/measurement_time
				latestMeas=rate_meas
				if latestMeas >= fieldMax[0][2]:
					endSim=True
				agent.belief_count[bin]+=1
				agent.belief_map[bin]= iterative_average(rate_meas,agent.belief_count[bin],round(agent.belief_map[bin],3))  #iteratively average rate measurement
				if len(agent.sensor.detectionSet)>0:
					#print("agent ",i,", rate = ",rate_meas,",average rate = ",agent.belief_map[bin], " in bin ", bin)
					#print(last_meas,t,dtSet)
					agent.sensor.detectionSet=set()	
		posx[i]=pos[0]
		posy[i]=pos[1]
	plt.clf()
	print(t,pos,u,latestMeas)
	if last_meas+measurement_time<=t:
		last_meas=t
	if field == fields[0]:
		for tag in taglist:
			if show_only_when_pinging:
				if tag.pinging(t):
					x,y,_ = tag.pos
					pinging_x=np.append(pinging_x,x)
					pinging_y=np.append(pinging_y,y)
			tag.updatePing(t)
		if show_only_when_pinging and visualize:
			draw((pinging_x,pinging_y))
		elif visualize:
			draw((tagx,tagy))
	if (field == fields[1] or field == fields[2] or field == fields[3]) and visualize:
		updateGP = True
		sensorRange=None
		plt.contourf(plottingPoints[:,:,0], plottingPoints[:,:,1],plottingPoints[:,:,2], 20, cmap='coolwarm')# cmap='inferno'), cmap='RdGy')
	if maxMeas<latestMeas:
		maxMeas=latestMeas
	t+=time_step
	if visualize:
		drawAgent((posx,posy),r=sensorRange)
	plt.pause(0.00001)#plt.pause(time_step)
	if endSim and stopOnMax:
		break
	
	
################################################ end simulation loop ####################################
################################################ final plots       ######################################
run=False
if method==searchMethods[0] or method==searchMethods[1]:
	sock.send("end \0".encode('utf-8'))
#input('done')
if field == fields[0]:
	draw((tagx,tagy))
	drawAgent((posx,posy),r=sensorRange)
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
if field == fields[1] or field == fields[2] or field == fields[3]:
	plt.contourf(plottingPoints[:,:,0], plottingPoints[:,:,1],plottingPoints[:,:,2], 20, cmap='coolwarm')# cmap='inferno'), cmap='RdGy')
	drawAgent((posx,posy))
	plt.figure(2)
	plt.axis('scaled')
	plt.grid(True)
	plt.contourf(plottingPoints[:,:,0], plottingPoints[:,:,1],plottingPoints[:,:,2], 20, cmap='coolwarm')# cmap='inferno'), cmap='RdGy')
	cbar = plt.colorbar()
	cbar.set_label('heat map')
	
	

plt.xlim([0, x_range])
plt.ylim([0, y_range])
plt.xticks(np.arange(0,x_range,spacing[0]))
plt.yticks(np.arange(0,y_range,spacing[1]))
plt.draw()
plt.pause(0.00001)
if logData:
	f=open("log.txt",'+a')
	f.write(field+","+str(t)+","+str(agent.getPos())+","+str(latestMeas)+"\n")
	f.close()
print(str(t)+","+str(agent.getPos())+","+str(latestMeas),", max val: ",maxMeas)
input('done')



