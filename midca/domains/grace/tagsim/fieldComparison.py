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

import traceback
import sys

wp_list = None

allDetectionData = None
allMeasurementData = []
t = 0
endSim = False			  
current_scale = None
current_offsets = None
start_ergodic_time = None
stop_ergodic_time = 18
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
def MidcaComLink():
    global running, searchComplete, wp_list, agent, E, det_count, agentList, off, sc, searchMIDCAErgodic
    run = True

    # create an INET, STREAMing socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # bind the socket to a public host, and a well-known port
    sock.bind(('127.0.0.1', 5700))
    sock.listen(5)
    # accept connections from outside
    (clientsocket, address) = sock.accept()
    # now do something with the clientsocket
    print("starting")
    while run:
        try:
            clientsocket, address = sock.accept()
            data = clientsocket.recv(1024)

            data = data.decode('utf-8')
            cmd = data.split(',')
            #print(cmd)
            if cmd[0] == 'quit':
                running = False
                run = False
                endSim = True
            if cmd[0] == 'start':
                running = True
            if cmd[0] == 'moveTo':
                x = int(cmd[1]) - 1
                y = int(cmd[2]) - 1
                center = np.array([x * x_range/5.0, y * y_range/5.0]) + np.array([.5*x_range/5.0, .5*y_range/5.0])
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
                #print(bin, bin2)
            if cmd[0] == 'time':
                clientsocket.send(str.encode(str(t)))
            if cmd[0] == 'getCell':
                agent = agentList[0]
                pos = agent.getPos()
                bin = E.getAbstractPos(pos[0], pos[1])
                myx, myy = E.getCellXY(pos[0], pos[1])
                clientsocket.send(str.encode(str(myx) + "," + str(myy)))
            if cmd[0] == 'search':
                x = int(cmd[1]) - 1
                y = int(cmd[2]) - 1
                center = np.array([x * x_range/5.0, y * y_range/5.0]) + np.array([.5*x_range/5.0, .5*y_range/5.0])
                wp_list[0] = search(wp_list[0], center)
                searchComplete = False

            if cmd[0] == 'searchErgodic':
                x = int(cmd[1]) - 1
                y = int(cmd[2]) - 1
                off = np.array([x * x_range/5.0, y * y_range/5.0])
                sc=x_range/5.0
                searchMIDCAErgodic=True
                if len(cmd)>=4:
                    sc = xrange if cmd[3]=='fullGrid' else x_range/5.0
                #searchComplete = False

            if cmd[0] == 'quitSearchErgodic':
                searchMIDCAErgodic=False
                searchComplete = True

            if cmd[0] == 'searchComplete':
                clientsocket.send(str.encode(str(searchComplete)))

            if cmd[0] == 'get_tags':
                agent = agentList[0]
                bin = 5 * (int(cmd[2]) - 1) + (int(cmd[1]))
                #print (bin)
                count = 0
                unique = []
                for data in allDetectionData:
                    if (data[3] == bin) and (not data[0] in unique):
                        count = count + 1
                        unique.append(data[0])
                clientsocket.send(str.encode(str(count)))

            if cmd[0] == 'get_measurement':
                #allMeasurementData.append([latestMeas, [pos[0], pos[1]], bin])
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

            if cmd[0] == "get_adjacent_measurement":
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
                        #print (data)
                        total_count += 1
                        unique.append(data[2])
                        # north
                        if data[2][1] > (yll + (factor * 1.50)):
                            count[0] += data[0]
                            measured_data[0].append(data[0])
                            time[0].append(data[1])
                        # print ("north")

                        # south
                        if data[2][1] < (yll + (factor *.50)):
                            count[1] += data[0]
                            measured_data[1].append(data[0])
                            time[1].append(data[1])
                        # print ("south")

                        # east
                        if data[2][0] > (xll + (factor *1.50)):
                            count[2] += data[0]
                            measured_data[2].append(data[0])
                            time[2].append(data[1])
                        # print ("east")

                        # west
                        if data[2][0] < (xll + (factor *.50)):
                            count[3] += data[0]
                            measured_data[3].append(data[0])
                            time[3].append(data[1])
                        # print ("west")
                #print ("time: ")
                #print (time)
                #print ("Measured_data : ")
                #print (measured_data)
                result = []


                # north
                avg_rate = find_max_7_values_avg_measurement(time[0], measured_data[0])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                # south
                avg_rate = find_max_7_values_avg_measurement(time[1], measured_data[1])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                # east
                avg_rate = find_max_7_values_avg_measurement(time[2], measured_data[2])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                # west
                avg_rate = find_max_7_values_avg_measurement(time[3], measured_data[3])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                data_to_be_sent = ",".join(str(i) for i in result)
                clientsocket.send(str.encode(data_to_be_sent))
            if cmd[0] == 'get_tags_adjacent':
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

                for data in allDetectionData:#tag ID,time,agent pos,bin
                    if (data[3] == bin) and (not data[0] in unique):
                        total_count += 1
                        unique.append(data[0])
                        # north
                        if data[2][1] > (yll + (factor * 1.50)):
                            count[0] += 1
                            time[0].append(data[1])
                        # print ("north")

                        # south
                        if data[2][1] (yll + (factor * .50)):
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

                print (time)
                print (count)
                result = []

                # north
                avg_rate = find_max_5_values_avg(time[0])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                # south
                avg_rate = find_max_5_values_avg(time[1])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                # east
                avg_rate = find_max_5_values_avg(time[2])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                # west
                avg_rate = find_max_5_values_avg(time[3])
                #print ("Average value")
                #print (avg_rate)
                result.append(avg_rate)

                data_to_be_sent = ",".join(str(i) for i in result)
                #print ("The data is : ")
                #print (data_to_be_sent)
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
            if cmd[0] == 'get_tags_adjacent_new':
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

            if cmd[0] == 'cell_lambda':
                agent = agentList[0]
                if len(cmd) < 2:
                    pos = agent.getPos()
                    bin = E.getAbstractPos(pos[0], pos[1]) - 1
                    clientsocket.send(str.encode(str(agent.belief_map[bin])))
                else:
                    #bin = E.getAbstractPos(int(cmd[1]), int(cmd[2])) - 1
                    bin = 5 * (int(cmd[1]) - 1) + (int(cmd[2])) - 1
                    clientsocket.send(str.encode(str(agent.belief_map[bin])))
                det_count[0] = 0
            clientsocket.close()#syncVar=True
        except Exception as e:
            traceback.print_exc()
            print(e)
    print("ending")
    clientsocket.close()

def ErgodicComLink():
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
        syncVar=True

def ErgodicComLink2():
    global u, run, updateGP,t
    run=True
    sock.send(str.encode(str(x_range)))
    while run:
        syncVar=False
        agent=agentList[0]
        st=agent.state
        bin=E.getAbstractPos(st[0],st[1])-1
        try:
            if not updateGP:
                sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(t)+" "+"None "))
            else:
                updateGP=False
                sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(t)+" "+str(latestMeas)))
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
            print(t,round(st[0],1),round(st[1],1),u)

def doubleIntegratorErgodicControl(agent,update):
    global run,t
    st=agent.state
    try:
        if not update:
            sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(st[2])+" "+str(st[3])+" "+str(t)+" "+"None "))
        else:
            sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(st[2])+" "+str(st[3])+" "+str(t)+" "+str(latestMeas)))
    except:
        run=False
        t=simtime
        return
    data = sock.recv(1024)
    data = data.decode('utf-8')
    cmd=data.split(',')
    if len(cmd)>1:
        u=(float(cmd[0]),float(cmd[1]))
        if st[0]>x_range or st[0]<0 or st[1]<0 or st[1]>y_range:
            _,utemp=wp_track(agent.getPos(),np.array([x_range/2,y_range/2]))
            u=np.clip(np.array([np.cos(utemp), np.sin(utemp)]),-0.1,0.1)
        return u
def doubleIntegratorErgodicControl(agent,update,scale=None,offsets=None):
    global run,t
    st=agent.state
    if scale!=None:
        sock.send(str.encode("change_scale "+str(scale)+"\n "))
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
            _,utemp=wp_track(agent.getPos(),np.array([[(offsets[0]+scale)/2,(offsets[1]+scale)/2]]))
            u=np.clip(np.array([np.cos(utemp), np.sin(utemp)]),-0.5,0.5)
        return u

def singleIntegratorErgodicControl(agent,update,scale=None,offsets=None):
    global run,t,current_scale,current_offsets
    st=agent.state
    print (offsets)

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
    print(data)
    if len(cmd)>1:
        u=(float(cmd[0]),float(cmd[1]))
        if st[0]>offsets[0]+scale or st[0]<offsets[0] or st[1]<offsets[1] or st[1]>offsets[1]+scale:
            _,utemp=wp_track(agent.getPos(),np.array([[offsets[0]+scale/2.0,offsets[1]+scale/2.0]]))
            u=np.clip(np.array([np.cos(utemp), np.sin(utemp)]),-1,1)
        return u

def MidcaIntegrator(agent,update):
    global running, searchComplete, wp_list, E, det_count, agentList, off, sc, searchMIDCAErgodic, t, start_ergodic_time
    run = True
    st= agent.state
    try:
        if not update:
            #sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(t)+" "+"None "))
            pass
        else:
            pass
            #sock.send(str.encode(str(round(st[0],1)/x_range)+" "+str(round(st[1],1)/x_range)+" "+str(t)+" "+str(latestMeas)))
    except:
        run=False
        t=simtime
        return
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
        print ("The time started is" + str(t))
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

    elif cmd[0] == 'quitSearchErgodic':
        searchMIDCAErgodic = False
        searchComplete = True

    elif cmd[0] == 'searchComplete':
        if searchComplete:
            print ("The time stopped is" + str(t))
        clientsocket.send(str.encode(str(searchComplete)))

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
        u=u+.27
        return 1*np.array([np.cos(u), np.sin(u)])
def m1_stepDrift2(x,u):#small remora attack on other wing or lost wing
        u=u-.27
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
simtime=500 #max simulation time
numAgents=1 #number of agents exploring
sensorRange=2
x_range=20.0 #grid size
y_range=20.0
spacing=(1,1)#(.5,.5) #spacing between points for visualizing fields
searchMethods = ["MIDCA","SUSD","ERGODIC_DI","DEMO","ERGODIC_SI"]
method = searchMethods[4]
fields= ["tag","gassian sum","rosenbrock","rastrigin"]
#fieldMax = [(5.5,14,1.5),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #100 and sensorRange = 2
#fieldMax = [(5.5,14,4),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #500 and sensorRange = 2
#fieldMax = [(5.5,14,10),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #500 and sensorRange = 2.5
#fieldMax = [(5.5,14,8),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #500 and sensorRange = 1.5
fieldMax = [(5.5,14,7),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #1000 and sensorRange = 2
#fieldMax = [(5.5,14,14),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #1000 and sensorRange = 2.5
#fieldMax = [(5.5,14,5),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #1000 and sensorRange = 2.5

field = fields[0]
fieldname="/Users/sravyakondrakunta/Documents/git/GracegridMIDCA/midca/domains/nbeacons/tagsim/tags_1000"
measurement_time = 2.0
time_step=.5
#start_pos=(.95*x_range,.9*y_range)#(.05*x_range,.1*y_range)#
"""
start_pos = [
             (4.361675414742551382e+00, 1.458277069766090328e+01),
             (1.545820006278236569e+01, 6.457247090829543623e+00),
             (6.295868290928718913e-01, 7.231587833833630796e+00),
             (1.125840547944832792e+01, 4.275635236417141272e-01),
             (1.547922926485634232e+00, 4.900255414078060312e+00),
             (1.510704267728113237e+01, 1.783478215173388648e+01),
             (2.204755346558018303e-01, 6.173268062869025741e+00),
             (4.958469349586305697e+00, 4.716507947006558510e+00),
             (1.479491107030286301e+01, 1.459564048874953457e+01),
             (3.171471926407209985e+00, 1.005076470181938575e+01),
             (1.234584097168078287e+01, 6.461087057506860631e+00),
             (1.067132595610898349e+01, 1.024013704378120160e+01),
             (2.627585809888457469e+00, 1.260149797724160514e+01),
             (4.046709532503120599e+00, 1.067246776102524386e+01),
             (1.408113055270808722e+01, 1.423915807887335561e+01),
             (1.414334725656964054e+01, 1.920331742981397838e+01),
             (8.702805949851414979e+00, 1.927219991502269991e+01),
             (1.902459420257870448e+01, 1.459444585671132621e+01),
             (1.071787563199973192e+01, 6.014617171762742132e+00),
             (7.813659480510352751e+00, 3.109500212981364253e+00)
             ]
"""
"""
start_pos = [
            (2.0, 2.0), (2.0, 6.0), (2.0, 10.0), (2.0, 14.0), (2.0, 18.0),
            (6.0, 2.0), (6.0, 6.0), (6.0, 10.0), (6.0, 14.0), (6.0, 18.0),
            (10.0, 2.0), (10.0, 6.0), (10.0, 10.0), (10.0, 14.0), (10.0, 18.0),
            (14.0, 2.0), (14.0, 6.0), (14.0, 10.0), (14.0, 14.0), (14.0, 18.0),
            (18.0, 2.0), (18.0, 6.0), (18.0, 10.0), (18.0, 14.0), (18.0, 18.0)
            ]
"""

start_pos = [
            (3.75, 3.75), (3.75, 7.75), (3.75, 11.75), (3.75, 15.75), (3.75, 19.75),
            (7.75, 3.75), (7.75, 7.75), (7.75, 11.75), (7.75, 15.75), (7.75, 19.75),
            (11.75, 3.75), (11.75, 7.75), (11.75, 11.75), (11.75, 15.75), (11.75, 19.75),
            (15.75, 3.75), (15.75, 7.75), (15.75, 11.75), (15.75, 15.75), (15.75, 19.75),
            (19.75, 3.75), (19.75, 7.75), (19.75, 11.75), (19.75, 15.75), (19.75, 19.75),
            (0.25, 0.25), (0.25, 4.25), (0.25, 8.25), (0.25, 12.25), (0.25, 16.25),
            (4.25, 0.25), (4.25, 4.25), (4.25, 8.25), (4.25, 12.25), (4.25, 16.25),
            (8.25, 0.25), (8.25, 4.25), (8.25, 8.25), (8.25, 12.25), (8.25, 16.25),
            (12.25, 0.25), (12.25, 4.25), (12.25, 8.25), (12.25, 12.25), (12.25, 16.25),
            (16.25, 0.25), (16.25, 4.25), (16.25, 8.25), (16.25, 12.25), (16.25, 16.25),
            (17.25, 18.25), (19.25, 3.25), (15.25, 8.25), (9.25, 10.25), (2.25, 1.25)
            ]

start_pos = start_pos[int(sys.argv[1])]
show_only_when_pinging=False
stopOnMax = True
visualize = True
syncronize = True
logData=True
###############################################################################################

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
    taglist= E.loadTagList(fieldname) #E.setMap(density_map)
    tagData=np.genfromtxt(fieldname+".csv",delimiter=",")
    #E.saveTagList()
for i in range(numAgents):
    s= AcousticReciever(np.array([0,0,0]),sensorRange)
    if method == searchMethods[2]:
        #agentList.append(Agent(np.array([np.random.rand()*x_range,np.random.rand()*y_range,0,0]),s,E,dim=2))
        agentList.append(Agent(np.array([start_pos[0],start_pos[1],0,0]),s,E,dim=2))
        agentList[i].dynamics=m2_step
        u=[0,0]
    elif method == searchMethods[4]:
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
midcasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
if method == searchMethods[2] or method == searchMethods[4]:
    # connect to ergodic controller
    sock.connect(('localhost', 5701))
if method == searchMethods[2] or method == searchMethods[4] and syncronize:	
    sock.send(str.encode(str(x_range)))


if method == searchMethods[2] and not syncronize:
    xthread = threading.Thread(target=ErgodicComLink)
    xthread.start()
if method == searchMethods[4] and not syncronize:
    xthread = threading.Thread(target=ErgodicComLink2)
    xthread.start()
if method == searchMethods[0] and syncronize:
    # now do something with the clientsocket
    # in this case, we'll pretend this is a threaded server
    midcasock.bind(('127.0.0.1', 5700))
    sock.connect(('localhost', 5701))
    midcasock.listen(5)
if method == searchMethods[0] and not syncronize:
    # now do something with the clientsocket
    # in this case, we'll pretend this is a threaded server
    xthread = threading.Thread(target=MidcaComLink)
    xthread.start()
    while not running:
        pass

##########################################################################
if method==searchMethods[3] or method==searchMethods[0]:
    wp_list = [[],[],[]]
    #wp_list[0] = search(wp_list[0], [.3*x_range, .3*y_range])
    #wp_list[0] = search(wp_list[0], [.3*x_range, .7*y_range])
    #wp_list[0] = search(wp_list[0], [5.5, 14])
if method==searchMethods[3] or method==searchMethods[0]:
    wp_list = [[],[],[]]
    for i in range(len(agentList)):
        agent=agentList[i]
        pos=agent.getPos()
        wp_list[i] = [[pos[0], pos[1]]]


det_count=[0,0,0]
################################################ simulation loop ####################################
endSim=False
maxMeas=0
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
        #srange=agent.sensor.range
        if (method==searchMethods[3] or method==searchMethods[0]) and not searchMIDCAErgodic:
            wp_list[i], u = wp_track(np.array(pos), wp_list[i])
            MidcaIntegrator(agent, updateGP)
            # print(t, pos, u, latestMeas)
            if updateGP:
                updateGP = False

        if method == searchMethods[2] and syncronize:
            u=doubleIntegratorErgodicControl(agent,updateGP)
            if updateGP:
                updateGP=False
        if method == searchMethods[4] and syncronize:
            u=singleIntegratorErgodicControl(agent,updateGP)
            if updateGP:
                updateGP=False
        if searchMIDCAErgodic:
            #off=(round(np.random.rand()*4)*x_range/5.0,round(np.random.rand()*4)*x_range/5.0)
            #sc=x_range/5.0
            if (t - start_ergodic_time) > stop_ergodic_time:
                searchMIDCAErgodic = False
                searchComplete = True
            u=singleIntegratorErgodicControl(agent,updateGP,scale=sc,offsets=off)
            if u[0]==0 and u[1]==0:
                _,u=wp_track(agent.getPos(),[np.array([(off[0]+sc)/2,(off[1]+sc)/2])])
            else:
                u=np.arctan2(u[1],u[0])
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

                # to stop in cell
                #pos = agent.getPos()
                #myx, myy = E.getCellXY(pos[0], pos[1])
                #if myx == 2 and myy == 4:
                #    endSim = True

                agent.belief_count[bin]+=1
                agent.belief_map[bin]= iterative_average(rate_meas,agent.belief_count[bin],round(agent.belief_map[bin],3))  #iteratively average rate measurement
                if len(agent.sensor.detectionSet)>0:
                    #print("agent ",i,", rate = ",rate_meas,",average rate = ",agent.belief_map[bin], " in bin ", bin)
                    #print(last_meas,t,dtSet)
                    agent.sensor.detectionSet=set()
        posx[i]=pos[0]
        posy[i]=pos[1]
    plt.clf()
    #print(t,pos,u,latestMeas)
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
if method==searchMethods[2] or method==searchMethods[4]:
    sock.send("end ".encode('utf-8'))
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
    f=open("log.txt",'a')
    f.write(field+","+str(t)+","+str(agent.getPos())+","+str(latestMeas)+"\n")
    f.close()
print(str(t)+","+str(agent.getPos())+","+str(latestMeas),", max val: ",maxMeas)
print('done')



