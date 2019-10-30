# -*- coding: utf-8 -*-
#!/usr/bin/python
__author__ = 'avumali'

import sys
import numpy as np

class ConditionalProbabilityTable:
    def __init__(self, table):
        self.table = table

class Node:
    def __init__(self, name,):
        self.name = name
        self.parents = []
        self.children = []
        self.value = None
    def addParent(self, parent):
        self.parents.append(parent)
    def addChild(self, child):
        self.children.append(child)
    def setCondProbTable(self, table):
        self.condProbTable = table
    def setValue(self, value):
        self.value = value

class Edge:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        start.addChild(end)
        end.addParent(start)


class BayesNet:

    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges
    def setTransitionModel(self, trans):
        self.trans = trans
    def setSensorModel(self, sense):
        self.sense = sense
    def setInitialProb(self, initProb):
        self.initProb = initProb

#this is the main function where this script begins to execute

inf = ['',0]

umbrella_obs = []
net_list = []

curNodes = []
curEdges = []

queryNode = None
bNet = None
evidenceList = []

samples = 0

def readFile():

    global inf
    global curNodes
    global curEdges
    global bNet
    global queryNode
    global  evidenceList
    global samples

    f = open('umbrella.txt', 'r')
    i = 0
    for line in f:
        inf[i] = line.rstrip();
        i+=1

    line = inf[0].split(',')
    samples = int(inf[1])
    
    umbrella_obs = np.zeros(len(line))
    for j in range(0,len(line)):
        if (line[j] == 't'):
            umbrella_obs[j] = True
        else:
            umbrella_obs[j] = False
        
    print umbrella_obs

    for j in range(0,len(line)):
        rain = Node('Rain')
        umbrella = Node('Umbrella')
        ruEdge = Edge(rain,umbrella)
        bNet = BayesNet([rain, umbrella], [ruEdge])
        net_list.append(bNet)
        
    print net_list


if __name__ == "__main__":
    
    readFile()


