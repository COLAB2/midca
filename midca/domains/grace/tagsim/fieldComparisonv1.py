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

allDetectionData = None


def find_max_5_values_avg(time):
    a = {}
    for each in time:
        if each in a:
            a[each] += 1
        else:
            a[each] = 1

    values = a.values()
    print ("best 5 values: ")
    if values:
        values.sort(reverse=True)
        print (values[:5])
        print (sum(values[:5]))
        return float(sum(values[:5])) / 5
    else:
        return 0


def MidcaComLink():
    global running, searchComplete, wp_list, agent, E, det_count, agentList, allDetectionData
    run = True
    # create an INET, STREAMing socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # bind the socket to a public host, and a well-known port
    sock.bind(('127.0.0.1', 5700))
    sock.listen(5)
    # accept connections from outside
    (clientsocket, address) = sock.accept()
    # now do something with the clientsocket

    while run:
        try:
            clientsocket, address = sock.accept()
            data = clientsocket.recv(2048)
            data = data.decode('utf-8')
            cmd = data.split(',')
            if cmd[0] == 'quit':
                running = False
                run = False
            if cmd[0] == 'start':
                running = True
            if cmd[0] == 'moveTo':
                x = int(cmd[1]) - 1
                y = int(cmd[2]) - 1
                center = np.array([x * 200, y * 200]) + np.array([100, 100])
                wp_list[0] = [center]
            if cmd[0] == 'moveToPhysicalPosition':
                x=int(cmd[1])-1
                y=int(cmd[2])-1
                center=np.array([x,y])
                wp_list[0]=[center]
            if cmd[0] == 'inCell':
                agent = agentList[0]
                pos = agent.getPos()
                bin = E.getAbstractPos(pos[0], pos[1])
                x = int(cmd[1])
                y = int(cmd[2])
                myx, myy = E.getCellXY(pos[0], pos[1])
                bin2 = 5 * (y - 1) + (x - 1)
                clientsocket.send(str.encode(str(x == myx and y == myy)))
                print(bin, bin2)

            if cmd[0] == 'getCell':
                agent = agentList[0]
                pos = agent.getPos()
                bin = E.getAbstractPos(pos[0], pos[1])
                myx, myy = E.getCellXY(pos[0], pos[1])
                clientsocket.send(str.encode(str(myx) + "," + str(myy)))

            if cmd[0] == 'search':
                x = int(cmd[1]) - 1
                y = int(cmd[2]) - 1
                center = np.array([x * 200, y * 200]) + np.array([100, 100])
                wp_list[0] = search(wp_list[0], center)
                searchComplete = False

            if cmd[0] == 'searchComplete':
                clientsocket.send(str.encode(str(searchComplete)))

            if cmd[0] == 'get_tags':
                agent = agentList[0]
                bin = 5 * (int(cmd[2]) - 1) + (int(cmd[1]))
                print (bin)
                count = 0
                unique = []
                for data in allDetectionData:
                    if (data[3] == bin) and (not data[0] in unique):
                        count = count + 1
                        unique.append(data[0])
                clientsocket.send(str.encode(str(count)))

            if cmd[0] == 'get_measurement':
                clientsocket.send(str.encode(str(latestMeas)))

            if cmd[0] == 'get_tags_adjacent':
                agent = agentList[0]
                xll = (int(cmd[1]) - 1) * 200
                yll = (int(cmd[2]) - 1) * 200
                pos = agent.getPos()
                bin = E.getAbstractPos(pos[0], pos[1])
                probability = []
                unique = []
                count = [0, 0, 0, 0]
                time = [[], [], [], []]
                total_count = 0

                for data in allDetectionData:
                    if (data[3] == bin) and (not data[0] in unique):
                        total_count += 1
                        unique.append(data[0])
                        # north
                        if data[2][1] > yll + 150:
                            count[0] += 1
                            time[0].append(data[1])
                        # print ("north")

                        # south
                        if data[2][1] < yll + 50:
                            count[1] += 1
                            time[1].append(data[1])
                        # print ("south")

                        # east
                        if data[2][0] > xll + 150:
                            count[2] += 1
                            time[2].append(data[1])
                        # print ("east")

                        # west
                        if data[2][0] < xll + 50:
                            count[3] += 1
                            time[3].append(data[1])
                        # print ("west")

                print (time)
                print (count)
                result = []

                # north
                avg_rate = find_max_5_values_avg(time[0])
                print ("Average value")
                print (avg_rate)
                result.append(avg_rate)

                # south
                avg_rate = find_max_5_values_avg(time[1])
                print ("Average value")
                print (avg_rate)
                result.append(avg_rate)

                # east
                avg_rate = find_max_5_values_avg(time[2])
                print ("Average value")
                print (avg_rate)
                result.append(avg_rate)

                # west
                avg_rate = find_max_5_values_avg(time[3])
                print ("Average value")
                print (avg_rate)
                result.append(avg_rate)

                data_to_be_sent = ",".join(str(i) for i in result)
                clientsocket.send(str.encode(data_to_be_sent))

                """
                #south
                avg_rate = find_max_5_values_avg(time[1])
                result.append(poisson_rate(avg_rate, 0.8))

                #east
                avg_rate = find_max_5_values_avg(time[2])
                result.append(poisson_rate(avg_rate, 0.8))

                #west
                avg_rate = find_max_5_values_avg(time[3])
                result.append(poisson_rate(avg_rate, 0.8))


                data_to_be_sent = ",".join(str(result))
                clientsocket.send(str.encode(data_to_be_sent))




                # calculate time
                for i,each in enumerate(time):
                    each.sort()
                    counted_times = sum([times-each[0] for times in each])
                    time[i] = counted_times

                print (time)
                print (count)

                result = []
                # calculate poison for north
                if time[0] and count[0]:
                    rate = count[0]/time[0]
                    p = math.exp(-rate)
                    for i in xrange(total_count):
                        p *= rate
                        p /= i + 1
                    result.append(p)
                else:
                    result.append(0.0)

                # calculate poison for south
                if time[1] and count[1]:
                    rate = count[1]/time[1]
                    p = math.exp(-rate)
                    for i in xrange(total_count):
                        p *= rate
                        p /= i + 1
                    result.append(p)
                else:
                    result.append(0.0)

                # calculate poison for east
                if time[2] and count[2]:
                    rate = count[2]/time[2]
                    p = math.exp(-rate)
                    for i in xrange(total_count):
                        p *= rate
                        p /= i + 1
                    result.append(p)
                else:
                    result.append(0.0)


                # calculate poison for west
                if time[3] and count[3]:
                    rate = count[3]/time[3]
                    p = math.exp(-rate)
                    for i in xrange(total_count):
                        p *= rate
                        p /= i + 1
                    result.append(p)
                else:
                    result.append(0.0)

                print (result)
                data_to_be_sent = ",".join(str(result))
                clientsocket.send(str.encode(data_to_be_sent))
                """

            if cmd[0] == 'cell_lambda':
                print allDetectionData
                agent = agentList[0]
                if len(cmd) < 2:
                    pos = agent.getPos()
                    bin = E.getAbstractPos(pos[0], pos[1]) - 1
                    clientsocket.send(str.encode(str(agent.belief_map[bin])))
                else:
                    # bin=E.getAbstractPos(int(cmd[1]),int(cmd[2]))-1
                    bin = 5 * (int(cmd[2]) - 1) + (int(cmd[1])) - 1
                    clientsocket.send(str.encode(str(agent.belief_map[bin])))

            clientsocket.close()
        except Exception as e:
            print(e)


def ErgodicComLink():
    global u, run, updateGP, t
    run = True
    sock.send(str.encode(str(x_range)))
    while run:
        agent = agentList[0]
        st = agent.state
        bin = E.getAbstractPos(st[0], st[1]) - 1
        try:
            if not updateGP:
                sock.send(str.encode(str(round(st[0], 1) / x_range) + " " + str(round(st[1], 1) / x_range) + " " + str(
                    st[2]) + " " + str(st[3]) + " " + str(t) + " " + "None "))
            else:
                updateGP = False
                sock.send(str.encode(str(round(st[0], 1) / x_range) + " " + str(round(st[1], 1) / x_range) + " " + str(
                    st[2]) + " " + str(st[3]) + " " + str(t) + " " + str(latestMeas)))
        except:
            run = False
            t = simtime
        data = sock.recv(1024)
        data = data.decode('utf-8')
        cmd = data.split(',')
        if len(cmd) > 1:
            u = (float(cmd[0]), float(cmd[1]))
            if st[0] > x_range or st[0] < 0 or st[1] < 0 or st[1] > y_range:
                _, utemp = wp_track(agent.getPos(), np.array([x_range / 2, y_range / 2]))
                u = np.clip(np.array([np.cos(utemp), np.sin(utemp)]), -0.1, 0.1)
            # print(st,u)
            print(t, round(st[0], 1), round(st[1], 1), round(st[2], 3), round(st[3], 3), u)


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
    plt.xticks(np.arange(0, E.x_range, E.x_range / 5.0))
    plt.yticks(np.arange(0, E.y_range, E.y_range / 5.0))

    plt.draw()


def drawAgent(x, r=None):
    plt.figure(1)
    plt.axis('scaled')
    plt.grid(True)
    plt.plot(x[0], x[1], 'bo')
    if r == None:
        pass
    else:
        circ = plt.Circle((x[0], x[1]), r, color='b', fill=False)
        plt.gcf().gca().add_artist(circ)
    plt.xlim([0, E.x_range])
    plt.ylim([0, E.y_range])
    plt.xticks(np.arange(0, E.x_range, E.x_range / 5.0))
    plt.yticks(np.arange(0, E.y_range, E.y_range / 5.0))

    plt.draw()


def iterative_average(x, n, ave):
    return ave + (x - ave) / (n + 1)


def simulate_dynamics(agent, u, tspan, dt):
    inc = agent.state
    for i in np.linspace(tspan[0], tspan[1], int((tspan[1] - tspan[0]) / dt)):
        inc += agent.dynamics(inc, u) * dt  # +world.flow(agent.getPos())*dt
    return inc


def f1_plan(x0, x, N):
    u = np.zeros(N)
    xsim = x0.copy()
    for n in range(N):
        e = x - xsim
        angle = np.arctan2(e[1], e[0])
        u[n] = angle
        xsim += 0.005 * np.array([np.cos(u[n]), np.sin(u[n])])

    return u


def search(wp_list, X):
    # X is the center position of one of the 25 cells
    offset = .375 * x_range / 5
    wp_list.append(np.array(X))
    wp_list.append(np.array(X) + np.array([-offset, offset]))
    wp_list.append(np.array(X) + np.array([offset, offset]))
    wp_list.append(np.array(X) + np.array([offset, -offset]))
    wp_list.append(np.array(X) + np.array([-offset, -offset]))
    wp_list.append(np.array(X))
    return wp_list


def wp_track(x, wp_list):
    global searchComplete
    e = np.array(wp_list[0]) - x
    if np.linalg.norm(e) < x_range / 50.0 and len(wp_list) > 1:
        # print(wp_list)
        del wp_list[0]

    if len(wp_list) == 1:
        searchComplete = True
    return wp_list, 1 * np.arctan2(e[1], e[0])


########################   motion models  ###################################
def m1_step(x, u):
    return 1 * np.array([np.cos(u), np.sin(u)])


def m2_step(x, u):
    #  |0 0   1    0|x        |0 0|
    #  |0 0   0    1|y     +  |0 0|u1
    #  |0 0  -a    0|vx       |1 0|u2
    #  |0 0   0   -b|vy       |0 1|
    a = .25
    return np.matmul(1 * np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, -a, 0], [0, 0, 0, -a]]), x) + np.matmul(
        1 * np.array([[0, 0], [0, 0], [1, 0], [0, 1]]), u)


def m3_step(x, u):
    return u[0] * np.array([np.cos(u[1]), np.sin(u[1])])


############################# test functions  ###############################################
def rastrigin(x, y):
    return 20 + x ** 2 + y ** 2 - 10 * (np.cos(2 * np.pi * x) + np.cos(2 * np.pi * y))


def rosenbrock(x, y):
    return 100 * (y - x ** 2) ** 2 + (1 - x) ** 2


def gaussianSum(x, y):
    r1 = np.array([.75 * x_range, .45 * y_range])
    r2 = np.array([.3 * x_range, .7 * y_range])
    loc = np.array([x, y])
    return 10 * np.exp(-0.05 * np.linalg.norm(loc - r1) ** 2) + 15 * np.exp(-0.1 * np.linalg.norm(loc - r2) ** 2)


density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
                        0.1, 0.3, 0.3, 0.1, 0.3,
                        0.2, 0.3, 0.3, 0.2, 0.1,
                        0.3, 0.9, 0.3, 0.2, 0.1,
                        0.2, 0.3, 0.2, 0.1, 0.1])
#################################### simulation settings   ###################################
N = 1000  # how many tags present
simtime = 1000  # max simulation time
numAgents = 1  # number of agents exploring
sensorRange = 2
x_range = 20.0  # grid size
y_range = 20.0
spacing = (1, 1)  # spacing between points for visualizing fields
searchMethods = ["MIDCA", "SUSD", "ERGODIC", "DEMO"]
method = searchMethods[0]
fields = ["tag", "gassian sum", "rosenbrock", "rastrigin"]
field = fields[1]
measurement_time = 2
time_step = .5
start_pos = (.05 * x_range, .1 * y_range)
###############################################################################################

t = 0
last_meas = t
run = False
running = False
searchComplete = False
updateGP = False
latestMeas = 0

taglist = []
agentList = []
tagx = np.zeros(N)
tagy = np.zeros(N)
for i in range(N):
    # taglist.append(AcousticTag(i,last_ping=np.random.randn()),ping_delay=max(2,30*np.random.randn())) # most realistic
    taglist.append(AcousticTag(i, last_ping=17 * np.random.randn()))  # more realistic (pings are not aligned in time)
    # taglist.append(AcousticTag(i)) #better for understanding because pings are aligned in time and  all have same ping interval
    x, y, _ = taglist[i].pos
    tagx[i] = x
    tagy[i] = y

E = Grid(taglist, x_range=x_range, y_range=y_range)
if field == fields[0]:
    taglist = E.loadTagList()  # E.setMap(density_map)
    # E.saveTagList()
for i in range(numAgents):
    s = AcousticReciever(np.array([0, 0, 0]), sensorRange)
    if method == searchMethods[2]:
        # agentList.append(Agent(np.array([np.random.rand()*x_range,np.random.rand()*y_range,0,0]),s,E,dim=2))
        agentList.append(Agent(np.array([start_pos[0], start_pos[1], 0, 0]), s, E, dim=2))
        agentList[i].dynamics = m2_step
        u = [0, 0]
    else:
        # agentList.append(Agent(np.array([np.random.rand()*x_range,np.random.rand()*y_range]),s,E,dim=2))
        agentList.append(Agent(np.array([start_pos[0], start_pos[1]]), s, E, dim=2))
        agentList[i].dynamics = m1_step

for i in range(N):
    x, y, _ = taglist[i].pos
    tagx[i] = x
    tagy[i] = y

if field == fields[1]:
    nx_bins = int(x_range / spacing[0])
    ny_bins = int(y_range / spacing[1])
    x_bins = np.array(range(nx_bins)) * x_range / nx_bins
    y_bins = np.array(range(ny_bins)) * y_range / ny_bins
    plottingPoints = [
        (idx + spacing[0] / 2.0, idy + spacing[1] / 2.0, gaussianSum(idx + spacing[0] / 2.0, idy + spacing[1] / 2.0))
        for idx in x_bins for idy in y_bins]
    plottingPoints = np.array(plottingPoints)
    plottingPoints.shape = (nx_bins, ny_bins, 3)
if field == fields[2]:
    nx_bins = int(x_range / spacing[0])
    ny_bins = int(y_range / spacing[1])
    x_bins = np.array(range(nx_bins)) * x_range / nx_bins
    y_bins = np.array(range(ny_bins)) * y_range / ny_bins
    plottingPoints = [
        (idx + spacing[0] / 2.0, idy + spacing[1] / 2.0, rosenbrock(idx + spacing[0] / 2.0, idy + spacing[1] / 2.0)) for
        idx in x_bins for idy in y_bins]
    plottingPoints = np.array(plottingPoints)
    plottingPoints.shape = (nx_bins, ny_bins, 3)
if field == fields[3]:
    nx_bins = int(x_range / spacing[0])
    ny_bins = int(y_range / spacing[1])
    x_bins = np.array(range(nx_bins)) * x_range / nx_bins
    y_bins = np.array(range(ny_bins)) * y_range / ny_bins
    plottingPoints = [
        (idx + spacing[0] / 2.0, idy + spacing[1] / 2.0, rastrigin(idx + spacing[0] / 2.0, idy + spacing[1] / 2.0)) for
        idx in x_bins for idy in y_bins]
    plottingPoints = np.array(plottingPoints)
    plottingPoints.shape = (nx_bins, ny_bins, 3)

# draw((tagx,tagy))
'''
for t in range(N):
    draw(taglist[t].pos)
   #plt.pause(.1)
'''
# simulation
# input('Enter to begin simulation')


########################################################################
# create an INET, STREAMing socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
if method == searchMethods[2]:
    # connect to ergodic controller
    sock.connect(('localhost', 8080))
    # now do something with the clientsocket
    # in this case, we'll pretend this is a threaded server
    xthread = threading.Thread(target=ErgodicComLink)
    xthread.start()

if method == searchMethods[0]:
    # now do something with the clientsocket
    # in this case, we'll pretend this is a threaded server
    xthread = threading.Thread(target=MidcaComLink)
    xthread.start()
##########################################################################
if method == searchMethods[3] or method == searchMethods[0]:
    wp_list = [[], [], []]
    wp_list[0] = search(wp_list[0], [.3 * x_range, .3 * y_range])

det_count = [0, 0, 0]
################################################ simulation loop ####################################
while t <= simtime:  # or running: #change to better simulation stopping criteria
    posx = np.zeros(numAgents)
    posy = np.zeros(numAgents)
    for i in range(len(agentList)):
        agent = agentList[i]
        pos = agent.getPos()
        # state = agent.getPos()#
        # srange=agent.sensor.range
        if method == searchMethods[3] or method==searchMethods[0]:
            wp_list[i], u = wp_track(np.array(pos), wp_list[i])
        state = simulate_dynamics(agent, u, [0, time_step], .1)
        dets = agent.updateAgent(state, t)
        allDetectionData = agent.sensor.detectionList  # history of every tag detection. includes (tag ID,time,agent pos,bin)
        det_count[i] += dets
        if field == fields[3]:
            latestMeas = rastrigin(pos[0], pos[1])
            if last_meas + measurement_time <= t:
                updateGP = True
        elif field == fields[2]:
            latestMeas = rosenbrock(pos[0], pos[1])
            if last_meas + measurement_time <= t:
                updateGP = True
        elif field == fields[1]:
            latestMeas = gaussianSum(pos[0], pos[1])
            if last_meas + measurement_time <= t:
                updateGP = True
        elif field == fields[0]:
            if last_meas + measurement_time <= t:
                updateGP = True
                bin = E.getAbstractPos(pos[0], pos[1]) - 1
                dtSet = agent.sensor.detectionSet
                rate_meas = len(dtSet) * 1.0 / measurement_time
                latestMeas = rate_meas
                agent.belief_count[bin] += 1
                agent.belief_map[bin] = iterative_average(rate_meas, agent.belief_count[bin],
                                                          round(agent.belief_map[bin],
                                                                3))  # iteratively average rate measurement
                if len(agent.sensor.detectionSet) > 0:
                    # print("agent ",i,", rate = ",rate_meas,",average rate = ",agent.belief_map[bin], " in bin ", bin)
                    # print(last_meas,t,dtSet)
                    agent.sensor.detectionSet = set()
        posx[i] = pos[0]
        posy[i] = pos[1]
    plt.clf()
    if last_meas + measurement_time <= t:
        last_meas = t
    if field == fields[0]:
        for tag in taglist:
            tag.updatePing(t)
        draw((tagx, tagy))
    if field == fields[1] or field == fields[2] or field == fields[3]:
        updateGP = True
        plt.contourf(plottingPoints[:, :, 0], plottingPoints[:, :, 1], plottingPoints[:, :, 2], 20,
                     cmap='coolwarm')  # cmap='inferno'), cmap='RdGy')

    t += time_step
    drawAgent((posx, posy), r=sensorRange)
    plt.pause(0.00001)  # plt.pause(time_step)
################################################ end simulation loop ####################################
################################################ final plots       ######################################
run = False
if method == searchMethods[2]:
    sock.send("end ".encode('utf-8'))

if field == fields[0]:
    for i in range(len(agentList)):
        agent = agentList[i]
        print("agent ", i, " rate estimates")
        agent.belief_map.shape = (5, 5)
        print(np.flip(agent.belief_map, 0))
        print("and measurements taken per cell")
        agent.belief_count.shape = (5, 5)
        print(np.flip(agent.belief_count, 0))

    print("True probability density  map")
    E.p.shape = (5, 5)
    print(np.flip(E.p, 0))
    # spacing=(50,50)
    print("Rate field approximation for sensor with range", sensorRange, " spaced at intervals of", spacing)
    approx, pnts = E.approximateField(measurement_time, spacing=spacing, sensorRange=sensorRange, get_points=True)
    # print(np.round(approx,decimals=2))
    plt.figure(2)
    plt.axis('scaled')
    plt.grid(True)
    # print('\n',pnts[:,:,0],'\n',pnts[:,:,1])
    # plt.plot(pnts[:,:,0].flatten(), pnts[:,:,1].flatten(), 'r.',cmap='coolwarm')
    plt.contourf(pnts[:, :, 0], pnts[:, :, 1], np.flip(np.round(approx, decimals=2), (0, 1)).transpose(), 20,
                 cmap='coolwarm')  # cmap='inferno'), cmap='RdGy')
    cbar = plt.colorbar()
    cbar.set_label('Detection rate')
if field == fields[1] or field == fields[2] or field == fields[3]:
    plt.figure(2)
    plt.axis('scaled')
    plt.grid(True)
    plt.contourf(plottingPoints[:, :, 0], plottingPoints[:, :, 1], plottingPoints[:, :, 2], 20,
                 cmap='coolwarm')  # cmap='inferno'), cmap='RdGy')
    cbar = plt.colorbar()
    cbar.set_label('heat map')

plt.xlim([0, x_range])
plt.ylim([0, y_range])
plt.xticks(np.arange(0, x_range, spacing[0]))
plt.yticks(np.arange(0, y_range, spacing[1]))
plt.draw()
plt.pause(0.00001)
input('done')



