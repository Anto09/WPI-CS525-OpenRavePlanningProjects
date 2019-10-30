# -*- coding: utf-8 -*-
#!/usr/bin/python
__author__ = 'avumali'

import sys
import  random
import numpy as np
import filtering
import pylab as pl

inf = ['',0]

umbrella_obs = []
net_list = []

curNodes = []
curEdges = []

queryNode = None
bNet = None
evidenceList = []

particles = 0

def readFile():

    global inf
    global curNodes
    global curEdges
    global bNet
    global queryNode
    global evidenceList
    global particles
    global umbrella_obs

    f = open('umbrella.txt', 'r')
    i = 0
    for line in f:
        inf[i] = line.rstrip();
        i+=1

    line = inf[0].split(',')
    particles = int(inf[1])
    
    umbrella_obs = np.zeros(len(line))
    for j in range(0,len(line)):
        if (line[j] == 't'):
            umbrella_obs[j] = True
        else:
            umbrella_obs[j] = False
        
    node_rain = filtering.Node('Rain')
    node_umbrella = filtering.Node('Umbrella')
    ru_Edge = filtering.Edge(node_rain, node_umbrella)

    curNodes = [node_rain, node_umbrella]
    curEdges = [ru_Edge]

    bNet = filtering.BayesNet(curNodes, curEdges)
    bNet.setInitialProb({True: 0.7})
    bNet.setTransitionModel({True: 0.7, False: 0.3})
    bNet.setSensorModel({True: 0.9, False: 0.2})
    
def ParticleFiltering (sampleSize):

    t = 0
    X = []
    W = np.zeros(sampleSize)
    
    X0 = []
    for m in range(0,sampleSize):
        r = random.random()
        if (r <= bNet.initProb[True]):
            X0.append(True)
        else:
            X0.append(False)
    X.append(X0)
    for z in umbrella_obs:
        t = t + 1
        St = []
        Xt = []
        for m in range (0, sampleSize):
            t_prob = bNet.trans[X[t-1][m]]
            ran = random.random()
            pr_val = False
            if (ran <= t_prob):
                pr_val = True
            s_prob = bNet.sense[pr_val]
            W[m] = s_prob
            St.append(tuple([pr_val, W[m]]))

        #resampling
        accepted = 0
        while (accepted < sampleSize):
            rand_index = random.randint(0,sampleSize-1)
            rand_weight = W[rand_index]
            rand_eval = random.random()
            if (rand_eval < rand_weight):
                Xt.append(St[rand_index][0])
                accepted = accepted + 1
        X.append(Xt)
    true_total = 0.0
    false_total = 0.0;
    for val in X[t]:
        if (val == True):
            true_total = true_total + 1.0
        else:
            false_total = false_total + 1.0
    total = true_total+false_total
    #print 'True',true_total/total,'False',false_total/total
    return [true_total/total, false_total/total]
            
if __name__ == "__main__":
    readFile()
    pf = ParticleFiltering(particles)
    print 'True',pf[0],'False',pf[1]
    '''
    data_true = []
    data_false = []
    runs = 30
    sample_particles = 0
    error_true = 999999
    error_false = 999999
    avg_true = 0
    avg_false = 0
    window = []
    lookBack = 10
    epsilon = 0.005
    while (error_true > epsilon or error_false > epsilon):
        sample_particles += 100
        run_results = []
        for i in range(0,runs):
            res = ParticleFiltering(sample_particles)
            run_results.append(tuple(res))
            avg_true = avg_true + run_results[i][0]
            avg_false = avg_false + run_results[i][1]
        avg_true = avg_true/runs
        avg_false = avg_false/runs
        print 'Avg:',avg_true,avg_false
        print 'Num particles',sample_particles
        data_true.append(tuple([avg_true, sample_particles]))
        data_false.append(tuple([avg_false, sample_particles]))
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
        print 'sample size', sample_particles

    print sample_particles
    
    x, y = zip(*data_true)
    
    pl.plot(y,x, color = "green")   
    pl.ylim(min(x),1.0)
    pl.show()
    
    w, z = zip(*data_false)
    pl.plot(z,w, color = "blue")
    
    pl.ylim(min(w),max(w))
    pl.show()
    '''