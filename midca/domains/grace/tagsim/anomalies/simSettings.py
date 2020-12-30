import numpy as np

density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
            0.1, 0.3, 0.3, 0.1, 0.3,
            0.2, 0.3, 0.3, 0.2, 0.1,
            0.3, 0.9, 0.3, 0.2, 0.1,
            0.2, 0.3, 0.2, 0.1, 0.1])
#################################### simulation settings   ###################################
ErgodicSocketInfo=('127.0.0.1', 5702)#('localhost', 8080)
MidcaSocketInfo=('127.0.0.1', 5700)
#N = 500 #how many tags present
simtime=600 #max simulation time
numAgents=1 #number of agents exploring
sensorRange=2 #sensor radius
x_range=20.0 #grid size
y_range=20.0 #grid size
spacing=(1,1)#(.5,.5) #spacing between points for visualizing fields
searchMethods = ["MIDCA","DEMO","ERGODIC"]
method = searchMethods[0]
anomaly_handling_methods = ["MIDCA", "None"]
anomaly_handling_method = anomaly_handling_methods[0]
fieldMax = [(5.5,14,6.3),(.3*x_range,.7*y_range,14)]#tag field absolute max 9.5 #100
fieldname="/home/sampath/Documents/git/midca/midca/domains/grace/tagsim/tags_1000"#"tags_1000
measurement_time = 2.0 #time used for estimating poisoon rate parameter
switchProb=1/100.0 #mode switch probability
rvwProb=100/100.0 #remora vs wing loss probability (higher means remora attack more likely)
remoraRemovalSuccess=1
RRDR=1 #remora removal decay rate (remora removal success multiplied by this constant every time remora attacks)
RSDR=.5 #remora speed decay rate (speed multiplied by this constant every time remora attacks)
time_step=.5 #simulation time step
downTime = 2.5/time_step #number of simulation steps you stay still after remora removal action
rng_seed = 100 #random seed number
#start_pos=(.95*x_range,.9*y_range)#(.05*x_range,.1*y_range)#

start_pos = [(4.361675414742551382e+00, 1.458277069766090328e+01),
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
             (7.813659480510352751e+00, 3.109500212981364253e+00),
            (2.0, 2.0), (2.0, 6.0), (2.0, 10.0), (2.0, 14.0), (2.0, 18.0),
            (6.0, 2.0), (6.0, 6.0), (6.0, 10.0), (6.0, 14.0), (6.0, 18.0),
            (10.0, 2.0), (10.0, 6.0), (10.0, 10.0), (10.0, 14.0), (10.0, 18.0),
            (14.0, 2.0), (14.0, 6.0), (14.0, 10.0), (14.0, 14.0), (14.0, 18.0),
            (18.0, 2.0), (18.0, 6.0), (18.0, 10.0), (18.0, 14.0), (18.0, 18.0),
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

show_only_when_pinging=False
stopOnMax = True
visualize = True
logData=True

###############################################################################################
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

def hotSpotProbability(rate_param,mean_rate_param,max_rate_param,tau=1,a=0.5):
    confidence=0
    k_eval=int(np.floor((a*mean_rate_param+(1-a)*max_rate_param))*tau)
    k=np.arange(k_eval)
    k_fact=np.zeros_like(k)
    for i in range(len(k_fact)):
        k_fact[i] = np.math.factorial(k[i])
    confidence = np.divide(np.power(rate_param,k)*np.exp(-rate_param),k_fact)
    return 1-confidence
