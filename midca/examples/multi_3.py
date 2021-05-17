results_file = "/home/zahid/Downloads/midca-update-to-python36/midca/examples/results_60:40_100_latest_2_rebllion"
b=100 #num of experiments
file = open (results_file, "r")
myfile = (file.readlines())
I=[]
J=[]
K=[]
for line in myfile:
    A=line.split(',')
    #print(A)
    I.append(int(A[1])) #list containining goals achived via rebellion
    J.append(int(A[3]))
    K.append(int(A[2]))#list containing goals achived by no rebellion
final_1=[]
final_2=[]
final_3=[]
for i in range(0,len(I),b):
    final_1.append(sum(I[i:i+b])/b)
    final_2.append(sum(J[i:i+b])/b)
    final_3.append(sum(K[i:i+b])/b)
for i in range(0,len(final_1)):
    final_1[i]= (final_1[i]/(2+i))*100
    final_2[i]= (final_2[i]/(2+i))*100
    final_3[i]= (final_3[i]/(2+i))*100


final_1 = [100]+final_1
final_2 = [100]+final_2
final_3= [0]+final_3
print(final_1)
print(final_2)
print(final_3)
A=list(range(1,21))
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
plt.figure()
plt.plot(A,final_1,label = "Rebel-agent")
plt.plot(A,final_2,label = "Standard-agent")
#plt.plot(A,final_3,label = 'Percentage of goals rebelled')
plt.xlabel('Goals given')
plt.ylabel('Percentage of goals achieved')
plt.xticks(np.arange(0, 22, 2.0))
plt.ylim(0,110)
plt.legend()
plt.show()

'''x=list(range(7,20))
for i in range(len(x)):
    final_1[i] = final_1[]'''