# -*- coding: utf-8 -*-
#!/usr/bin/python
__author__ = 'avumali'

import sys

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

#this is the main function where this script begins to execute

inf = ['',0]

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

    #filename = sys.argv[-1]
    #f = open(filename, 'r')
    f = open('inference.txt', 'r')    
    i = 0
    for line in f:
        inf[i] = line.rstrip();
        i+=1

    line = inf[0].split(',')

    cloudy = Node('Cloudy')
    sprinkler = Node('Sprinkler')
    rain = Node('Rain')
    wetGrass = Node('Wet Grass')

    if (line[0] == 't'):
        cloudy.value = True
        evidenceList.append(cloudy)
    elif (line[0] == 'f'):
        cloudy.value = False
        evidenceList.append(cloudy)
    elif (line[0] == 'q'):
        queryNode = cloudy

    if (line[1] == 't'):
        sprinkler.value = True
        evidenceList.append(sprinkler)
    elif (line[1] == 'f'):
        sprinkler.value = False
        evidenceList.append(sprinkler)
    elif (line[1] == 'q'):
        queryNode = sprinkler

    if (line[2] == 't'):
        rain.value = True
        evidenceList.append(rain)
    elif (line[2] == 'f'):
        rain.value = False
        evidenceList.append(rain)
    elif (line[2] == 'q'):
        queryNode = rain

    if (line[3] == 't'):
        wetGrass.value = True
        evidenceList.append(wetGrass)
    elif (line[3] == 'f'):
        wetGrass.value = False
        evidenceList.append(wetGrass)
    elif (line[3] == 'q'):
        queryNode = wetGrass

    csEdge = Edge(cloudy, sprinkler)
    crEdge = Edge(cloudy, rain)
    swEdge = Edge(sprinkler, wetGrass)
    rwEdge = Edge(rain, wetGrass)

    curNodes = [wetGrass, rain, sprinkler, cloudy]
    curEdges = [csEdge, crEdge, swEdge, rwEdge]

    cpt_cloudy = {None: 0.5}
    cpt_sprinkler = {(True,): 0.1, (False,): 0.5}
    cpt_rain = {(True,): 0.1, (False,): 0.2}
    cpt_wetGrass = {(True, True):0.99, (True, False):0.9, (False, True):0.9, (False, False):0.1}

    cloudy.setCondProbTable(cpt_cloudy)
    sprinkler.setCondProbTable(cpt_sprinkler)
    rain.setCondProbTable(cpt_rain)
    wetGrass.setCondProbTable(cpt_wetGrass)

    bNet = BayesNet(curNodes, curEdges)
    samples = int(inf[1])

if __name__ == "__main__":
    readFile()


