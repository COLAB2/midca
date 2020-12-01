#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#MIDCA API
# Created August 2019 by Danielle Brown via CoLab2
#Reads in a file of the format specification,object2,object1
#Specification can be a type, i.e. a spoon (object2) is a utensil (object1)
#or specification can be an attribute, i.e., a utensil (object2) can be a color (object1)
#Requirement: Be able to take in input and add it to the object hierarchy tree
#Requirement: Act as an interface between ros and MIDCA for objects

#To Do still:
# #Ideas from meeting with Mike Cox and Sampath Gogineni on 10/8/2019
#Detector -> type, xblock, isablock
#then it is passed to MIDCA from interface
#All objects have types and possibly attributes. Attributes have values (ie type color has values
#green, red, etc)
#uniform naming via the type system and simple attributes
#Each action has a piece of code to handle and determine when its over
#define predicate formulation
#Be Able to ultimately define a new domain
#  detect object types, what are predicates in domain, do actions, when is action done, and
#  naming of objects and handling of attributes
#  Naming -> type + unique identifier, decrement unique identifier if item disappears so
#  block2 doesn't become block3 when it reappears

#robot -> be able to list objects, then the robot detects objects, and can click to assign objects?

#TODO: Attribute and AttributeValue need to be separate things

import sys
import csv
import fileinput #for command line file name

from collections import defaultdict


directedGraph = defaultdict(list)
attributesAndValues = defaultdict(list)
enumerationList = []

#Adds node to the directedGraph
def addEdge(directedGraph, u, v):
    directedGraph[u].append(v)

#Adds the directional relationship between objects
def generate_edges(directedGraph):
    edges = []
    # for each node in directedGraph
    for node in directedGraph:
        # for each relation node of a single node
        for relation in directedGraph[node]:
            # if edge exists then append
            edges.append((node, relation))
    return edges


#box object class
def blockShape(self):
    return 0

#Plate object class
def plateShape(self):
    return 0

#fork object class
def forkShape(self):
    return 0

#spoon object class
def spoonShape(self):
    return 0

#function that prints out the enumeration list of attributes with index values
def printEnumerationList(enumerationList):
    for idx, (object, attribute) in enumerate(enumerationList):
        print("index is %d, name is %s, and attribute is %s"
              % (idx, object, attribute))

#main function that reads in a file of data and creates a directed tree of types
#and directed tree of attributes as well as an enumeration list of attributes
def objectInput(directedGraph, enumerationList, newFile2):
    index = 0
    #create directed graph tree off of information provided in document
    #newFile2 = open('test.txt')
    fileText2 = csv.reader(newFile2, delimiter = ',')
    for row in fileText2:
        #if it is a type it is added to the type tree
        if row[0] == "type":
             directedGraph[row[2]].insert(index, row[1])
             index += 1
         #if it is an attribute, then it addes it to the attribute tree
         #and enumeration list
        elif row[0] == "attribute":
            attributesAndValues[row[1]].insert(index, row[2])
            tempTupple = (row[2], row[1])
            enumerationList.append(tempTupple)

    #newFile2.close()
        

def printPDDL(directedGraph): 
    # print PDDL types
    print('\t(:types')
    for node in directedGraph:    
        #print('' + node)
        
        for relation in directedGraph[node]:
            print('\t ' +  relation + ' -' + node)
            
    print('\t)\n')
    
    
# prints tree with DFS
def printTree(directedGraph, node, printAttributesBool): 
    visited = set() # Set to keep track of visited nodes.
    attributes = attributesAndValues[node]
        
    dfs(visited, directedGraph, node, 0, printAttributesBool, attributes) 
        
        
def dfs(visited, graph, node, depth, printAttributesBool, attributes):
    if node not in visited:
        print('\t' * depth + '+-- ' + node)
        attributes = attributes + attributesAndValues[node]
        if printAttributesBool:
            #print('\t' * depth + '      ' + (str)(attributes))  # print all accumulated attributes
            print('\t' * depth + '      ' + (str)(attributesAndValues[node]))  # print only specific attributes
        visited.add(node)
        for child in graph[node]:
            dfs(visited, graph, child, depth + 1, printAttributesBool, attributes)


objectInput(directedGraph, enumerationList, fileinput.input()) #fileinput.input() is for passed cmd line files

# print generated trees and edges
#print("Edges of type tree: " + str(generate_edges(directedGraph)))
#print ("Type Tree: " + (str)(directedGraph))
#print("Edges of attribute tree: " + str (generate_edges(attributesAndValues)))
#print ("Attribute Tree: " + (str)(attributesAndValues))
#printEnumerationList(enumerationList)

# test type values from input
printPDDL(directedGraph)


#API ENFORCES ENTITY PARENT
printAttributesBool = True
printTree(directedGraph, 'entity', printAttributesBool)

