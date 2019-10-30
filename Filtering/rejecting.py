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

def priorSample(nodes):
    nodes.reverse()

    values = {}
    for node in nodes:
        prTrue = Pr(node,True,values)
        if random.uniform(0.0,1.0) <= prTrue:
            values[node] = True
            node.value = True
        else:
            values[node] = False
            node.value = False

    nodes.reverse()

    return values

def consistent(dict1, dict2):
    for elem in dict1:
        if elem in dict2 and dict1[elem] != dict2[elem]:
            return False
    return True

def rejectionSample(query,evidence,nodes,samples):
    N = {True:0, False:0}
    for i in range(0, samples):
        cur_nodes = copy.deepcopy(nodes)
        e = []
        X = None

        for j in range(0, len(evidence)):
            for node in cur_nodes:
                if (node.name == evidence[j].name):
                    e.append(node)
                    break

        for node in cur_nodes:
            if (node.name == query.name):
                X = node
                break
        sample = priorSample(cur_nodes)

        for elem in e:
            for o_elem in evidence:
                if (elem.name == o_elem.name):
                    elem.value = o_elem.value
                    break

        e_vals = dict();
        for item in e:
            e_vals.update({item: item.value})

        if consistent(sample,e_vals):
            N[sample[X]] += 1

    count = float(N[True] + N[False])
    if count <= .5:
        print('All values rejected')
        return None
    normalized_N = {True: N[True]/count, False: N[False]/count}
    return [normalized_N]

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
    print(rejectionSample(queryNode,evidenceList,curNodes,samples))
    '''
    data_true = []
    data_false = []
    epsilon = 0.01
    error_true = 9999999
    error_false = 9999999
    sample_size = 0
    prev_true = 0
    prev_false = 0
    lookBack = 10
    window = []
    runs = 10

    while (error_true > epsilon or error_false > epsilon):
        run_results = []
        sample_size += 100
        no_res = False
        div = 0
        for i in range(0,runs):
            div = div + 1
            avg_true = 0
            avg_false = 0
            run_results.append(rejectionSample(queryNode,evidenceList,curNodes,sample_size))
            if (run_results[i] is None):
                continue
            #print type(run_results[i])
            avg_true += run_results[i][0][True]
            avg_false += run_results[i][0][False]
        if (len(run_results) == 0):
                continue
        avg_true /= div
        avg_false /= div
        print avg_true, avg_false
        data_true.append(tuple([avg_true, sample_size]))
        data_false.append(tuple([avg_false, sample_size]))
        if (len(window) < lookBack):
            window.append(tuple([avg_true, avg_false]))
        else:
            error_true = 0
            error_false = 0
            for w in window:
                error_true = max(error_true, avg_true - w[0])
                error_false = max(error_false, avg_false - w[1])
            window.remove(window[0])
            window.append(tuple([avg_true, avg_false]))
        if (len(window) >= lookBack and error_true < 9999999):
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