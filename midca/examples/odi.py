import os

results_file = "/home/zahid/Downloads/midca-update-to-python36/midca/examples/results"
b=3
for j in range(b):
    os.system('python3 /home/zahid/Downloads/midca-update-to-python36/midca/examples/minecraft_ff_pddl.py')
#execfile('Downloads/midca-update-to-python36/midca/examples')
file = open (results_file, "r")
myfile = (file.readlines())
I=[]
J=[]
for line in myfile:
    A=line.split(',')
    #print(A)
    I.append(int(A[1]))
    J.append(int(A[2]))
final_1=[]
final_2=[]
for i in range(0,len(I),b):
    final_1.append(sum(I[i:i+b])/b)
    final_2.append(sum(J[i:i+b])/b)


print(final_1,final_2)
#print(I,J)
#mywordlist = myfile.split(',')
#print(mywordlist)
