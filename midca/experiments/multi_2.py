import os
from midca.examples import minecraft_ff_pddl
A=[], B=[], C=[]
for i in range(50):
    os.system('python3 /home/zahid/Downloads/midca-update-to-python36/midca/examples/minecraft_ff_pddl.py')
    a,b,c= minecraft_ff_pddl.xyz()
    A.append(a)
    B.append(b)
    c.append(c)
print(A,B,C)
#execfile('Downloads/midca-update-to-python36/midca/examples')