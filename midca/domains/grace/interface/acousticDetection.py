import numpy as np
from .AcousticTag import AcousticTag
from .gridworldAcoustic import Grid
from . import loc


def initialize_tags(N):
    #generate N tags and append to taglist
    taglist = []
    for i in range(N):
        delay=17#max(1,30*np.random.rand())
        #each tag object has an ID, an offset (last_ping), and a delay between pings (ping_delay)
        taglist.append(AcousticTag(i,last_ping=-delay*np.random.rand(),ping_delay=delay))
    return taglist

def generateTaglist(taglist, x_range=1600*5, y_range=-1600*4, fieldname="tags"):
    density_map = np.array([0.1, 0.1, 0.4, 0.2, 0.1,
						    0.5, 0.9, 0.6, 0.1, 0.3,
						    0.2, 0.6, 0.3, 0.2, 0.2,
                            0.2, 0.3, 0.3, 0.2, 0.3])
    E = Grid(taglist,x_range=x_range, y_range=y_range,xbins=4,ybins=3)
    E.setMap(density_map)
    E.saveTagList(fname=fieldname)

def loadTagList(fieldname="tags"):
    tagData=np.genfromtxt(fieldname+".csv",delimiter=",")
    return tagData

def tagField(tagData, pos, t, time_step, sensorRange=500):
    distance=np.linalg.norm(tagData[:,1:3]-np.array([pos[0],pos[1]]),axis=1)
    eps=time_step/100.0
    c1=(np.fmod(t,tagData[:,4]+eps)-(tagData[:,0]+tagData[:,4]))<time_step
    c2=(np.fmod(t,tagData[:,4]+eps)>(tagData[:,0]+tagData[:,4]))
    pinging = np.logical_and(c1,c2)
    dtSet= np.logical_and(distance<sensorRange,pinging)
    return tagData[np.where(pinging)[0],:],tagData[np.where(dtSet)[0],5],np.sum(dtSet)#pinging,detection set,detectionNum

def write_to_file(taglist, filename = "/home/sampath/moos-ivp/moos-ivp-midca/missions/genios/tags.txt"):

    f = open(filename, "w")
    for i, point in enumerate(taglist):
        point = [taglist[i][1], taglist[i][2]]
        label_count = int(taglist[i][5])
        #content = [b"AddHazard", b"x=" + str(point[0]) + ",y=" + str(point[1]) + ",label= " + str(self.label_count) + ", type=benign"]
        #self.publisher.send_multipart(content)
        content = "hazard = x=" + str(point[0]) + ",y=" + str(point[1]) + ",label= " + str(label_count) + ", type=benign\n"
        f.write(content)
    f.close()

def write_to_file_lat_long(taglist, filename = "tags_lat_long.txt"):

    f = open(filename, "w")
    for i, point in enumerate(taglist):
        point = [taglist[i][1], taglist[i][2]]
        label_count = int(taglist[i][5])
        #content = [b"AddHazard", b"x=" + str(point[0]) + ",y=" + str(point[1]) + ",label= " + str(self.label_count) + ", type=benign"]
        #self.publisher.send_multipart(content)
        x,y = loc.LocalGrid2LatLong(point[0], point[1])
        content = str(x) + "," + str(y) + "\n"
        #content = "hazard = x=" + str(point[0]) + ",y=" + str(point[1]) + ",label= " + str(label_count) + ", type=benign\n"
        f.write(content)
    f.close()

if __name__ == "__main__":
    #taglist = initialize_tags(1000)
    #generateTaglist(taglist)
    tagData = loadTagList()
    #write_to_file(tagData)
    write_to_file_lat_long(tagData)
    #pinging, detection_num, detection_set = tagField(tagData, [800,-5605], 15, 17)
    #print (detection_set)
