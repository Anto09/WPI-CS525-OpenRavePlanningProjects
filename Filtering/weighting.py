# -*- coding: utf-8 -*-
#!/usr/bin/python
__author__ = 'antoumali'

import  inference
import random
import math
import sys
import time
import copy
import numpy as np
import pylab as pl

def Pr(node, val, evidence):
    parents = node.parents
    if len(parents) == 0:
        truePr = node.condProbTable[None]
    else:
        parentVals = []
        for e in evidence:
            if (e in node.parents):
                parentVals.append(e.value)
        truePr = node.condProbTable[tuple(parentVals)]
    if val==True:
        return truePr
    else:
        return 1.0-truePr

def searchNode(node, list):
    for elem in list:
        if (node.name == elem.name):
            return True
    return False

def weightedSample(nodes, evidence):
    x = {}
    w = 1

    for node in nodes:
        if (searchNode(node,evidence)):
            if (len(node.parents) > 0):
                parentVals = []
                for p in node.parents:
                    parentVals.append(p.value)
                val = node.condProbTable[tuple(parentVals)]
                if (node.value == True):
                    w = w*val
                else:
                    w = w*(1-val)
            else:
                val = node.condProbTable[None]
                if (node.value == True):
                    w = w*val
                else:
                    w = w*(1-val)
        else:
            parent_vals = []
            if (len(node.parents) > 0):
                for p in node.parents:
                    parent_vals.append(p.value)
                true_pr = node.condProbTable[tuple(parent_vals)]
            else:
                true_pr = node.condProbTable[None]
            rand_flip = random.random()
            if (rand_flip <= true_pr):
                node.value = True
            else:
                node.value = False
        x.update({node: node.value})

    return (x,w)

def likelihoodWeighting(query,evidence,nodes,samples):
    W = {True:0, False:0}
    nodes.reverse()

    for j in range (0, samples):

        cur_nodes = copy.deepcopy(nodes)

        e = []
        for i in range(0, len(evidence)):
            for node in cur_nodes:
                if (node.name == evidence[i].name):
                    e.append(node)
                    break
        for elem in e:
            for o_elem in evidence:
                if (elem.name == o_elem.name):
                    elem.value = o_elem.value
                    break

        sample = weightedSample(curNodes,e)
        W[sample[0][query]] = W[sample[0][query]] +  sample[1]

    mag = W[True]+W[False]
    return {True: W[True]/mag, False: W[False]/mag}

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

    cloudy = inference.Node('Cloudy')
    sprinkler = inference.Node('Sprinkler')
    rain = inference.Node('Rain')
    wetGrass = inference.Node('Wet Grass')

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

    csEdge = inference.Edge(cloudy, sprinkler)
    crEdge = inference.Edge(cloudy, rain)
    swEdge = inference.Edge(sprinkler, wetGrass)
    rwEdge = inference.Edge(rain, wetGrass)

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

    bNet = inference.BayesNet(curNodes, curEdges)
    samples = int(inf[1])

if __name__ == "__main__":
    readFile()

    #t = time.clock()
    print(likelihoodWeighting(queryNode,evidenceList,curNodes,samples))
    '''
    data_true = []
    data_false = []
    epsilon = 0.01
    error_true = 9999999
    error_false = 9999999
    sample_size = 0
    prev_true = 0
    prev_false = 0
    lookback = 10
    window = []
    runs = 10

    while (error_true > epsilon or error_false > epsilon):
        run_results = []
        sample_size += 100
        for i in range(0,runs):
            avg_true = 0
            avg_false = 0
            run_results.append(likelihoodWeighting(queryNode,evidenceList,curNodes,sample_size))
            #print run_results[i]
            avg_true += run_results[i][True]
            avg_false += run_results[i][False]
        avg_true /= runs
        avg_false /= runs
        print avg_true, avg_false
        data_true.append(tuple([avg_true, sample_size]))
        data_false.append(tuple([avg_false, sample_size]))
        if (len(window) < lookback):
            window.append(tuple([avg_true, avg_false]))
        else:
            error_true = 0
            error_false = 0
            for w in window:
                error_true = max(error_true, avg_true - w[0])
                error_false = max(error_false, avg_false - w[1])
            window.remove(window[0])
            window.append(tuple([avg_true, avg_false]))
        if (len(window) >= lookback and error_true < 9999999):
            print 'error',error_true,error_false
        print 'sample size', sample_size

    print sample_size

    x, y = zip(*data_true)
    
    pl.plot(y,x, color = "green")   
    pl.ylim(min(x),max(x))
    pl.show()
    
    w, z = zip(*data_false)
    pl.plot(z,w, color = "blue")
    
    pl.ylim(min(w),max(w))
    pl.show()
    '''
    #print time.clock() - t