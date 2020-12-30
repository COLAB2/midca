
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
import simSettings as cfg
#import simSettings100x100 as cfg

import traceback
import sys

p_list = None

allDetectionData = None
allMeasurementData = []
endSim = False			  
current_scale = None
current_offsets = None
modes = ["m1", "m2", "m3", "m4"]
mode = None
start_ergodic_time = None
stop_ergodic_time = None
static_ergodic_time = 18
anomaly_count = 0
anomaly_history = []


def find_max_5_values_avg(time):
    a = {}
    for each in time:
        if each in a:
            a[each] += 1
        else:
            a[each] = 1

    values = a.values()
    #print ("best 5 values: ")
    if values:
        values.sort(reverse=True)
        #print (values[:5])
        #print (sum(values[:5]))
        return float(sum(values[:7])) / 7
    else:
        return 0

def find_max_7_values_avg_measurement(time, data):
    a = {}
    for index, each in enumerate(time):
        if each in a:
            a[each] += data[index]
        else:
            a[each] = data[index]

    values = a.values()
    #print ("best 5 values: ")
    if values:
        values.sort(reverse=True)
        #print (values[:5])
        #print (sum(values[:5]))
        return float(sum(values[:5])) / 5
    else:
        return 0
      
      
def MidcaIntegrator(agent,update):
    global running, searchComplete, wp_list, E, det_count, agentList, off, sc, searchMIDCAErgodic, start_ergodic_time, removeRemoraAction
    run = True
    # accept connections from outside
    (clientsocket, address) = midcasock.accept()
    data = clientsocket.recv(1024)
    data = data.decode('utf-8')
    cmd=data.split(',')
    #print(cmd)
    if cmd[0] == 'quit':
        running = False
        run = False
        endSim = True
    elif cmd[0] == 'start':
        running = True
    elif cmd[0] == 'moveTo':
        x = int(cmd[1]) - 1
        y = int(cmd[2]) - 1
        center = np.array([x * x_range / 5.0, y * y_range / 5.0]) + np.array(
            [.5 * x_range / 5.0, .5 * y_range / 5.0])
        wp_list[0] = [center]
    elif cmd[0] == 'moveToPhysicalPosition':
        x = int(cmd[1]) - 1
        y = int(cmd[2]) - 1
        center = np.array([x, y])
        wp_list[0] = [center]
    elif cmd[0] == 'inCell':
        agent = agentList[0]
        pos = agent.getPos()
        bin = E.getAbstractPos(pos[0], pos[1])
        x = int(cmd[1])
        y = int(cmd[2])
        myx, myy = E.getCellXY(pos[0], pos[1])
        bin2 = 5 * (y - 1) + (x - 1)
        clientsocket.send(str.encode(str(x == myx and y == myy)))
        # print(bin, bin2)
    elif cmd[0] == 'time':
        clientsocket.send(str.encode(str(t)))
    elif cmd[0] == 'getCell':
        agent = agentList[0]
        pos = agent.getPos()
        bin = E.getAbstractPos(pos[0], pos[1])
        myx, myy = E.getCellXY(pos[0], pos[1])
        clientsocket.send(str.encode(str(myx) + "," + str(myy)))
    elif cmd[0] == 'search':
        x = int(cmd[1]) - 1
        y = int(cmd[2]) - 1
        center = np.array([x * x_range / 5.0, y * y_range / 5.0]) + np.array(
            [.5 * x_range / 5.0, .5 * y_range / 5.0])
        wp_list[0] = search(wp_list[0], center)
        searchComplete = False

    elif cmd[0] == 'searchErgodic':
        print (cmd)
        x = int(cmd[1]) - 1
        y = int(cmd[2]) - 1
        off = np.array([x * x_range / 5.0, y * y_range / 5.0])
        sc = x_range / 5.0
        searchMIDCAErgodic = True
        if len(cmd) >= 4:
            sc = xrange if cmd[3] == 'fullGrid' else x_range / 5.0
        start_ergodic_time = t
        # searchComplete = False

    elif cmd[0] == 'searchErgodicComplete':
        clientsocket.send(str.encode(str(not(searchMIDCAErgodic))))

    elif cmd[0] == 'quitSearchErgodic':
        searchMIDCAErgodic = False
        searchComplete = True

    elif cmd[0] == 'searchComplete':
        clientsocket.send(str.encode(str(searchComplete)))

    elif cmd[0] == 'getMode':
        clientsocket.send(str.encode(str(mode)))

    elif cmd[0] == 'removeRemora':
        removeRemoraAction = True

    elif cmd[0] == 'removeRemoraStatus':
        if removeRemoraAction == True:
            clientsocket.send(str.encode(str(False)))
        else:
            clientsocket.send(str.encode(str(True)))

    elif cmd[0] == 'get_tags':
        agent = agentList[0]
        bin = 5 * (int(cmd[2]) - 1) + (int(cmd[1]))
        # print (bin)
        count = 0
        unique = []
        for data in allDetectionData:
            if (data[3] == bin) and (not data[0] in unique):
                count = count + 1
                unique.append(data[0])
        clientsocket.send(str.encode(str(count)))

    elif cmd[0] == 'get_measurement':
        # allMeasurementData.append([latestMeas, [pos[0], pos[1]], bin])
        # latestMeas, [pos0, pos1], bin
        bin = 5 * (int(cmd[2]) - 1) + (int(cmd[1])) - 1
        print (bin)
        sum = 0
        unique = []
        for data in allMeasurementData:
            if (data[3] == bin) and (not data[2] in unique):
                sum += data[0]
                unique.append(data[2])
                clientsocket.send(str.encode(str(sum)))

    elif cmd[0] == "get_adjacent_measurement":
        agent = agentList[0]
        factor = 2
        xll = (int(cmd[1]) - 1) * factor * 2
        yll = (int(cmd[2]) - 1) * factor * 2
        pos = agent.getPos()
        bin = E.getAbstractPos(pos[0], pos[1]) - 1
        unique = []
        count = [0, 0, 0, 0]
        time = [[], [], [], []]
        measured_data = [[], [], [], []]
        total_count = 0

        for data in allMeasurementData:  # tag ID,time,agent pos,bin
            if (data[3] == bin) and (not data[2] in unique):
                # print (data)
                total_count += 1
                unique.append(data[2])
                # north
                if data[2][1] > (yll + (factor * 1.50)):
                    count[0] += data[0]
                    measured_data[0].append(data[0])
                    time[0].append(data[1])
                # print ("north")

                # south
                if data[2][1] < (yll + (factor * .50)):
                    count[1] += data[0]
                    measured_data[1].append(data[0])
                    time[1].append(data[1])
                # print ("south")

                # east
                if data[2][0] > (xll + (factor * 1.50)):
                    count[2] += data[0]
                    measured_data[2].append(data[0])
                    time[2].append(data[1])
                # print ("east")

                # west
                if data[2][0] < (xll + (factor * .50)):
                    count[3] += data[0]
                    measured_data[3].append(data[0])
                    time[3].append(data[1])
                # print ("west")
        # print ("time: ")
        # print (time)
        # print ("Measured_data : ")
        # print (measured_data)
        result = []

        # north
        avg_rate = find_max_7_values_avg_measurement(time[0], measured_data[0])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        # south
        avg_rate = find_max_7_values_avg_measurement(time[1], measured_data[1])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        # east
        avg_rate = find_max_7_values_avg_measurement(time[2], measured_data[2])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        # west
        avg_rate = find_max_7_values_avg_measurement(time[3], measured_data[3])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        data_to_be_sent = ",".join(str(i) for i in result)
        clientsocket.send(str.encode(data_to_be_sent))
    elif cmd[0] == 'get_tags_adjacent':
        agent = agentList[0]
        factor = 2
        xll = (int(cmd[1]) - 1) * factor * 2
        yll = (int(cmd[2]) - 1) * factor * 2
        pos = agent.getPos()
        bin = E.getAbstractPos(pos[0], pos[1])
        probability = []
        unique = []
        count = [0, 0, 0, 0]
        time = [[], [], [], []]
        total_count = 0

        for data in allDetectionData:  # tag ID,time,agent pos,bin
            if (data[3] == bin) and (not data[0] in unique):
                total_count += 1
                unique.append(data[0])
                # north
                if data[2][1] > (yll + (factor * 1.50)):
                    count[0] += 1
                    time[0].append(data[1])
                # print ("north")

                # south
                if data[2][1] < (yll + (factor * .50)):
                    count[1] += 1
                    time[1].append(data[1])
                # print ("south")

                # east
                if data[2][0] > (xll + (factor * 1.50)):
                    count[2] += 1
                    time[2].append(data[1])
                # print ("east")

                # west
                if data[2][0] < (xll + (factor * .50)):
                    count[3] += 1
                    time[3].append(data[1])
                # print ("west")

        #print (time)
        #print (count)
        result = []

        # north
        avg_rate = find_max_5_values_avg(time[0])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        # south
        avg_rate = find_max_5_values_avg(time[1])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        # east
        avg_rate = find_max_5_values_avg(time[2])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        # west
        avg_rate = find_max_5_values_avg(time[3])
        # print ("Average value")
        # print (avg_rate)
        result.append(avg_rate)

        data_to_be_sent = ",".join(str(i) for i in result)
        # print ("The data is : ")
        # print (data_to_be_sent)
        clientsocket.send(str.encode(data_to_be_sent))

    elif cmd[0] == 'get_tags_adjacent_new':
        agent = agentList[0]
        factor = 2
        xll = (int(cmd[1]) - 1) * factor * 2
        yll = (int(cmd[2]) - 1) * factor * 2
        pos = agent.getPos()
        bin = E.getAbstractPos(pos[0], pos[1])
        probability = []
        unique = []
        count = [0, 0, 0, 0]
        time = [[], [], [], []]
        total_count = 0

        for data in allDetectionData:  # tag ID,time,agent pos,bin
            if (data[3] == bin) and (not data[0] in unique):
                total_count += 1
                unique.append(data[0])
                # north
                if data[2][1] > (yll + (factor * 1.50)):
                    count[0] += 1
                    time[0].append(data[1])
                # print ("north")

                # south
                if data[2][1] < (yll + (factor * .50)):
                    count[1] += 1
                    time[1].append(data[1])
                # print ("south")

                # east
                if data[2][0] > (xll + (factor * 1.50)):
                    count[2] += 1
                    time[2].append(data[1])
                # print ("east")

                # west
                if data[2][0] < (xll + (factor * .50)):
                    count[3] += 1
                    time[3].append(data[1])
            # print ("west")

        print (count)

        data_to_be_sent = ",".join(str(i) for i in count)
        # print ("The data is : ")
        # print (data_to_be_sent)
        clientsocket.send(str.encode(data_to_be_sent))

    elif cmd[0] == 'cell_lambda':
        agent = agentList[0]
        if len(cmd) < 2:
            pos = agent.getPos()
            bin = E.getAbstractPos(pos[0], pos[1]) - 1
            clientsocket.send(str.encode(str(agent.belief_map[bin])))
        else:
            # bin = E.getAbstractPos(int(cmd[1]), int(cmd[2])) - 1
            bin = 5 * (int(cmd[1]) - 1) + (int(cmd[2])) - 1
            clientsocket.send(str.encode(str(agent.belief_map[bin])))
        det_count[0] = 0

    clientsocket.close()

    if len(cmd)>=1:
        return

      
def singleIntegratorErgodicControl(agent,update,scale=None,offsets=None):
    global run,t,current_scale,current_offsets
    st=agent.state
    #print (offsets)
    if scale!=None:
        if current_scale != scale:
            sock.send(str.encode("change_scale "+str(scale)))
            time.sleep(1)
            sock.send(str.encode("change_scale " + str(scale)))
            confirm= sock.recv(1024)
            current_scale = scale if "confirm" in confirm.decode('utf-8') else x_range
    else:
        scale=x_range

    if not offsets is None:
        if not current_offsets is offsets:
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
        traceback.print_exc()
        #print(e)
        run=False
        t=simtime
        return
    data = sock.recv(1024)
    data = data.decode('utf-8')
    cmd=data.split(',')
    #print(data)
    if len(cmd)>1:
        u=(float(cmd[0]),float(cmd[1]))
        if st[0]>offsets[0]+scale or st[0]<offsets[0] or st[1]<offsets[1] or st[1]>offsets[1]+scale:
            _,utemp=wp_track(agent.getPos(),np.array([[offsets[0]+scale/2.0,offsets[1]+scale/2.0]]))
            u=np.clip(np.array([np.cos(utemp), np.sin(utemp)]),-1,1)
            print("out of bounds")
        return u
######################################  helper functions ##################################      
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

def search(wp_list,X):
    #X is the center position of one of the 25 cells
    offset=.375*x_range/5
    wp_list.append(np.array(X))
    wp_list.append(np.array(X)+np.array([-offset,offset]))
    wp_list.append(np.array(X)+np.array([offset,offset]))
    wp_list.append(np.array(X)+np.array([offset,-offset]))
    wp_list.append(np.array(X)+np.array([-offset,-offset]))
    wp_list.append(np.array(X) + np.array([-offset, offset]))
    wp_list.append(np.array(X))
    return wp_list

def wp_track(x,wp_list):
    global  searchComplete
    e = np.array(wp_list[0])-x
    if np.linalg.norm(e) < x_range/50.0 and len(wp_list) > 1:
        #print(wp_list)
        del wp_list[0]

    if len(wp_list) == 1:
        searchComplete=True
    return wp_list, 1*np.arctan2(e[1],e[0])

########################   motion models  ###################################                  
def m1_step(x,u):
        return 1*np.array([np.cos(u), np.sin(u)])
def m1_stepHalfSpeed(x,u):#big remora attack
        return .5*np.array([np.cos(u), np.sin(u)])
def m1_stepDrift1(x,u):#small remora attack on one wing or lost wing
        u=u+.37
        return 0.5*np.array([np.cos(u), np.sin(u)])
def m1_stepDrift2(x,u):#small remora attack on other wing or lost wing
        u=u-.37
        return 0.5*np.array([np.cos(u), np.sin(u)])  
  

simtime=cfg.simtime 
numAgents=cfg.numAgents 
sensorRange=cfg.sensorRange
x_range=cfg.x_range
y_range=cfg.y_range
spacing=cfg.spacing
searchMethods = cfg.searchMethods
anomaly_handling_method = cfg.anomaly_handling_method
method = cfg.method
fieldMax = cfg.fieldMax
fieldname=cfg.fieldname
measurement_time = cfg.measurement_time
time_step= cfg.time_step
switchProb=cfg.switchProb
if len(sys.argv)>1:
	start_pos = cfg.start_pos[int(sys.argv[1])]
else:
	start_pos = cfg.start_pos[0]
removalRate=cfg.remoraRemovalSuccess
if cfg.rng_seed != None:
	np.random.seed(cfg.rng_seed)
t=0
last_meas=t
run=False
running=False
searchComplete=False
updateGP=False
searchMIDCAErgodic=False
latestMeas=0
sc=x_range
off=(0,0)
u=0
faultyMode=False
reasons=["remora","wing"]
explanation=""
removeRemoraAction=False
taglist=[]
agentList=[]


E = Grid(taglist,x_range=x_range, y_range=y_range)
taglist= E.loadTagList(fieldname) #E.setMap(density_map)
tagData=np.genfromtxt(fieldname+".csv",delimiter=",")
tagx=tagData[:,1]
tagy=tagData[:,2]
N=len(taglist)
for i in range(numAgents):
    s= AcousticReciever(np.array([0,0,0]),sensorRange)
    agentList.append(Agent(np.array([start_pos[0],start_pos[1]]),s,E,dim=2))
    agentList[i].dynamics=m1_step
    agentList[i].remora = 1    
        
        
  #########################  socket threads ###############################################
# create an INET, STREAMing socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
midcasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
midcasock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if method == searchMethods[2] :
    # connect to ergodic controller
    sock.connect(cfg.ErgodicSocketInfo)
    sock.send(str.encode(str(x_range)))
    if anomaly_handling_method == "MIDCA":
        midcasock.bind(('127.0.0.1', 5700))
        midcasock.listen(5)


if method == searchMethods[0] :
    # now do something with the clientsocket
    # in this case, we'll pretend this is a threaded server
    midcasock.bind(('127.0.0.1', 5700))
    sock.connect(('localhost', 5701))
    midcasock.listen(5)
    
if method==searchMethods[1] :
    wp_list = [[] for k in range(numAgents)]
    for i in range(len(agentList)):
        agent=agentList[i]
        pos=agent.getPos()
        center = np.array([4 * x_range / 5.0, 4* y_range / 5.0]) + np.array([.5 * x_range / 5.0, .5 * y_range / 5.0])
        wp_list[i]=search(wp_list[i],center)   
if method==searchMethods[0]:
    wp_list = [[] for k in range(numAgents)]
    for i in range(len(agentList)):
        agent=agentList[i]
        pos=agent.getPos()
        wp_list[i] = [[pos[0], pos[1]]]
det_count=[[0] for k in range(numAgents)]
################################################ simulation loop ####################################

endSim = False
maxMeas = 0
lost_steps = [0 for k in range(numAgents)]
stuck = [(0, 0) for k in range(numAgents)]
while t <= simtime:  # or running:
    posx = np.zeros(numAgents)
    posy = np.zeros(numAgents)
    # print(t)
    for i in range(len(agentList)):
        agent = agentList[i]
        pos = agent.getPos()
        temp = 1 if faultyMode else np.random.rand()
        # print(temp)
        if removeRemoraAction:
            if lost_steps[i] == 0:
                stuck[i] = pos
            lost_steps[i] += 1
            # print(lost_steps[i])
            if lost_steps[i] >= cfg.downTime:
                if np.random.rand() < removalRate and explanation != reasons[1]:
                    agent.dynamics = m1_step
                    mode = modes[0]
                    stop_ergodic_time = static_ergodic_time
                    faultyMode = False
                    explanation = ""
                    print(t, "removing remora sucessful")
                lost_steps[i] = 0
                removeRemoraAction = False
        if temp < switchProb:
            faultyMode = True
            if temp / switchProb < 1 / 3.0:
                agent.dynamics = m1_stepHalfSpeed
                explanation = reasons[0]
                mode = modes[3]
                anomaly_count += 1
                anomaly_history.append(mode)
                stop_ergodic_time = static_ergodic_time + 0.7 *static_ergodic_time
            elif temp / switchProb < 2 / 3.0:
                agent.dynamics = m1_stepDrift1
                explanation = reasons[cfg.rvwProb < np.random.rand()]
                mode = modes[2]
                anomaly_count += 1
                anomaly_history.append(mode)
                stop_ergodic_time = static_ergodic_time + 0.5 *static_ergodic_time
            else:
                agent.dynamics = m1_stepDrift2
                explanation = reasons[cfg.rvwProb < np.random.rand()]
                mode = modes[1]
                anomaly_count += 1
                anomaly_history.append(mode)
                stop_ergodic_time = static_ergodic_time + 0.5 *static_ergodic_time

        if (method == searchMethods[0]) and not searchMIDCAErgodic:
            wp_list[i], u = wp_track(np.array(pos), wp_list[i])
            MidcaIntegrator(agent, updateGP)
            #print(t, pos, u, latestMeas)
            if updateGP:
                updateGP = False
        if method == searchMethods[1]:
            wp_list[i], u = wp_track(np.array(pos), wp_list[i])
        if method == searchMethods[2]:
            u = singleIntegratorErgodicControl(agent, updateGP)
            if anomaly_handling_method == "MIDCA":
                MidcaIntegrator(agent, updateGP)
            if u[0] == 0 and u[1] == 0:
                _, u = wp_track(agent.getPos(), [np.array([(off[0] + sc) / 2, (off[1] + sc) / 2])])
            else:
                u = np.arctan2(u[1], u[0])

            if updateGP:
                updateGP = False

        if searchMIDCAErgodic:
            # off=(round(np.random.rand()*4)*x_range/5.0,round(np.random.rand()*4)*x_range/5.0)
            # sc=x_range/5.0
            if (t - start_ergodic_time) > stop_ergodic_time:
                searchMIDCAErgodic = False
            u = singleIntegratorErgodicControl(agent, updateGP, scale=sc, offsets=off)
            MidcaIntegrator(agent, updateGP)
            if u[0] == 0 and u[1] == 0:
                _, u = wp_track(agent.getPos(), [np.array([(off[0] + sc) / 2, (off[1] + sc) / 2])])
            else:
                u = np.arctan2(u[1], u[0])
                if updateGP:
                    updateGP = False
        if removeRemoraAction:
            _, u = wp_track(np.array(pos), [np.array(stuck[i])])
        state = simulate_dynamics(agent, u, [0, time_step], .1)
        agent.updateAgent(state, t)
        pos = agent.getPos()
        pinging, detSet, dets = cfg.tagField(tagData, pos, t, time_step, sensorRange)
        agent.updateAgent(state, t, dets=detSet)
        # print(t,pinging.shape,dets,dets,detSet,agent.sensor.detectionSet)
        allDetectionData = agent.sensor.detectionList  # history of every tag detection. includes (tag ID,time,agent pos,bin)
        det_count[i] += dets
        if last_meas + measurement_time <= t:
            updateGP = True
            bin = E.getAbstractPos(pos[0], pos[1]) - 1
            dtSet = agent.sensor.detectionSet
            rate_meas = len(dtSet) * 1.0 / measurement_time
            latestMeas = rate_meas
            if latestMeas >= fieldMax[0][2]:
                endSim = True
            # to stop in cell
            # pos = agent.getPos()
            # myx, myy = E.getCellXY(pos[0], pos[1])
            # if myx == 2 and myy == 4:
            #    endSim = True
            agent.belief_count[bin] += 1
            agent.belief_map[bin] = iterative_average(rate_meas, agent.belief_count[bin], round(agent.belief_map[bin],
                                                                                                3))  # iteratively average rate measurement
            if len(agent.sensor.detectionSet) > 0:
                agent.sensor.detectionSet = set()
        posx[i] = pos[0]
        posy[i] = pos[1]
    plt.clf()
    # print(t,pos,u,latestMeas)
    if last_meas + measurement_time <= t:
        last_meas = t

    if cfg.show_only_when_pinging and cfg.visualize:
        draw((pinging[:, 1], pinging[:, 2]))
    elif cfg.visualize:
        draw((tagx, tagy))
    if maxMeas < latestMeas:
        maxMeas = latestMeas
    t += time_step
    if cfg.visualize:
        drawAgent((posx, posy), r=sensorRange)
    plt.pause(0.00001)  # plt.pause(time_step)
    if endSim and cfg.stopOnMax:
        break

################################################ end simulation loop ####################################
################################################ final plots       ######################################
run=False
if method==searchMethods[2]:
    sock.send("end ".encode('utf-8'))
    
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

plt.xlim([0, x_range])
plt.ylim([0, y_range])
plt.xticks(np.arange(0,x_range,spacing[0]))
plt.yticks(np.arange(0,y_range,spacing[1]))
plt.draw()
plt.pause(0.00001)

if cfg.logData:
    f = open(method + "log.txt", 'a')
    f.write(str(t) + "," + str(agent.getPos()) + "," + str(latestMeas) + "," + str(anomaly_count) + "," + str(anomaly_history) +"\n")
    f.close()
print(str(t)+","+str(agent.getPos())+","+str(latestMeas),", max val: ",maxMeas)
print('done')
