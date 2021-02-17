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

from __future__ import print_function
import sys
import csv
import fileinput  # supports inplace editing
import argparse  # handles cmd line args and verbose logging levels
import shutil  # file copying

from collections import defaultdict


directedGraph = defaultdict(list)
attributes = defaultdict(list)
enumerationList = []
attributeValues = defaultdict(list)
relations = defaultdict(list)
entityRelations = defaultdict(list)  # associates relations to the key of entity

# all inherited values for each entity
inheritedAttributes = defaultdict(list)
inheritedRelations = defaultdict(list)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Graph Functions
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
def objectInput(directedGraph, enumerationList, config):
    index = 0
    #create directed graph tree off of information provided in document
    configFile = open(config)
    fileText = csv.reader(configFile, delimiter = ',')
    for row in fileText:
        #if it is a type it is added to the type tree
        if row[0] == "type":
             directedGraph[row[2]].insert(index, row[1])
             index += 1
         #if it is an attribute, then add it to the attribute tree
         #and enumeration list
        elif row[0] == "attribute":
            attributes[row[1]].insert(index, row[2])
            tempTupple = (row[2], row[1])
            enumerationList.append(tempTupple)
        elif row[0] == "attributeValue":
            attributeValues[row[1]].insert(index, row[2])
        elif row[0] == "relation":
            for type in row[2:]:  # for each type in the relationship
                relations[row[1]].insert(index, type)
            entityRelations[row[2]].append(row[1])  # add relation as a child of the first entity

    configFile.close()
        

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
    attrib = attributes[node]
        
    dfsPrint(visited, directedGraph, node, 0, printAttributesBool, attrib) 
        
        
def dfsPrint(visited, graph, node, depth, printAttributesBool, attrib):
    if node not in visited:
        print('\t' * depth + '+-- ' + node)
        attrib = attrib + attributes[node]
        if printAttributesBool:
            print('\t' * depth + '      ' + (str)(attributes[node]))  # print only specific attributes
        visited.add(node)
        for child in graph[node]:
            dfsPrint(visited, graph, child, depth + 1, printAttributesBool, attrib)
            
# accumulate inherited attributes with DFS
def accumulateAttributes(directedGraph, node): 
    visited = set() # Set to keep track of visited nodes.
    attrib = attributes[node]
    eRelations = entityRelations[node]
        
    dfsAccumulate(visited, directedGraph, node, attrib, eRelations) 
        
        
def dfsAccumulate(visited, graph, node, attrib, eRelations):
    if node not in visited:
        attrib = attrib + attributes[node]
        eRelations = eRelations + entityRelations[node]
        visited.add(node)
        
        #only save accumulated attributes for leaf nodes
        if len(graph[node]) == 0: 
            inheritedAttributes[node] = attrib
            inheritedRelations[node] = eRelations
            #print('{}: {}'.format(node, inheritedAttributes[node]))
            #print('{}: {}'.format(node, inheritedRelations[node]))
            
        for child in graph[node]:
            dfsAccumulate(visited, graph, child, attrib, eRelations)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Writing Templates
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def modifyDetectors(filename):
    file = fileinput.FileInput(filename, inplace=True) 
    #file = fileinput.FileInput(filename)  # test output

    for line in file: 
        if '<domain>' in line: 
            print(line.replace('<domain>', filename.replace('Detectors.cpp', '')), end='')
            
        # Add methods for each entity in domain
        elif '<methods>' in line:
            for attrib in inheritedAttributes:
                methodTxt = 'std::string detect_<entity>()\n{\n\t//TODO: create <entity>_attr string with the following attributes:\n\t//<attributes>\n\n\treturn <entity>_attr;\n}\n'
                methodTxt = methodTxt.replace('<entity>', attrib)
                methodTxt = methodTxt.replace('<attributes>', (str)(inheritedAttributes[attrib]))
                
                print(methodTxt)
        # Add topics for each entity in domain
        elif '<topic>' in line:
            for attrib in inheritedAttributes:
                topicTxt = '\tros::Publisher <entity>_pub = n.advertise<std_msgs::String>("<entity>_attr", 1);'
                topicTxt = topicTxt.replace('<entity>', attrib)
                
                print(topicTxt)
        # Add publishers for each entity in domain
        elif '<publish>' in line:
            for attrib in inheritedAttributes:
                publishTxt = '\t\t<entity>_pub.publish(<entity>_attr);'
                publishTxt = publishTxt.replace('<entity>', attrib)
                
                print(publishTxt)
                
        else:
            print(line, end='')

    file.close()
    
def modifyHandler(filename):
    file = fileinput.FileInput(filename, inplace=True) 
    #file = fileinput.FileInput(filename)  # test output
    #print(' ')

    for line in file: 
        if '<domain>' in line: 
            print(line.replace('<domain>', filename.replace('EntitiesHandler.py', '')), end='')
            
        # Add relations for each entity in domain
        elif '<relations>' in line:
            for relate in inheritedRelations:
                print('        #{}: {}'.format(relate, (str)(inheritedRelations[relate])))
        else:
            print(line, end='')

    file.close()    

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Main
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == "__main__":
    # manage args
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="domain specific configuration file")
    parser.add_argument("-v","--verbosity",action="count",default=0,help="increase output verbosity")
    args = parser.parse_args()
    
    objectInput(directedGraph, enumerationList, args.config) #generate config from passed file 
    
    # add output based on verbosity
    if args.verbosity >= 1:
        # print generated trees and edges
        print("Edges of type tree: " + str(generate_edges(directedGraph)))
        if args.verbosity >= 2:
            print("Type Tree: " + (str)(directedGraph))

        print("\nEdges of attribute tree: " + str (generate_edges(attributes)))
        if args.verbosity >= 2:
            print("Attribute Tree: " + (str)(attributes))
        # printEnumerationList(enumerationList)

        print("\nEdges of attributeValues tree: " + str (generate_edges(attributeValues)))
        if args.verbosity >= 2:
            print("Attribute Tree: " + (str)(attributeValues))

        print("\nEdges of relations tree: " + str (generate_edges(relations)))
        if args.verbosity >= 2:
            print("Relations Tree: " + (str)(relations))

        # test type values from input
        # printPDDL(directedGraph)
    if args.verbosity == 3:
        #API ENFORCES ENTITY PARENT
        print('')
        printTree(directedGraph, 'entity', False)  # print entity tree without attributes
    elif args.verbosity >= 4:
        #API ENFORCES ENTITY PARENT
        print('')
        printTree(directedGraph, 'entity', True)  # print entity tree with attributes
        
        
    accumulateAttributes(directedGraph, 'entity')
    
    # Test output for each entity with their attributes for object detection code stubs and helpers
    # copy template to modify
    src='templates/detectorsTemplate.txt'
    dst= args.config.replace('.txt', 'Detectors.cpp')
    shutil.copy(src,dst)
    
    modifyDetectors(dst)

    
    # Test output for each entity and their predicates for perception to determine relations

    # copy template to modify
    src='templates/entitiesHandlerTemplate.txt'
    dst= args.config.replace('.txt', 'EntitiesHandler.py')
    shutil.copy(src,dst)
    
    modifyHandler(dst)















