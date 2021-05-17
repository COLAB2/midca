#with open("sas_plan","r") as fileha:
#	line = fileha.readline()
#	while line:
##		A = line.strip("\n")
#		print(A)
#		print(A.strip(" "))
#
#		line = fileha.readline()



#f = open("sas_plan","r")
#A= f.read().split()
#print(A)
#for i in range(len(A)):
#	A[i] = A[i].replace("(","")
#	A[i] = A[i].replace(")","")

#print(A[:-6])
#A=A[:-6]
#print(A)

#A = [(A[i],A[i+1],A[i+2]) for i in range(0,len(A),3)]
#it = iter(A)

#print(zip(it,it,it))

#print(A)
import os,subprocess

subprocess.run(["python3","fast_downward.py","plant-protection-modif.pddl","plant-pro_modif.pddl","--search","lazy_greedy([ff])"])
