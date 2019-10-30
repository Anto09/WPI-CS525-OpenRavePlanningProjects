# -*- coding: utf-8 -*-
#!/usr/bin/python
__author__ = 'antoumali'

import random
import math
import Gnuplot
import sys
import time
import copy
import  Queue
import numpy as np
import  collections

data = []
results = []

def readFile():
    with open('data.txt') as openfileobject:
        for line in openfileobject:
            line = line.split(' ')
            data.append(tuple([float(line[0]), float(line[1]), float(line[2])]))

def euclideanDistance((a,b,c), (d,e,f)): #first arg = test point, second arg = sameple point
    return pow(d-a,2) + pow(e-b,2)

def KNN(n, point,limit, samples):
    classifier = {0:0, 1:0}
    max_dist = 0;
    max_point = None
    radius_points = []
    for x in range (0, limit):
        dist = euclideanDistance(point,samples[x])
        if (len(radius_points) < n):
            radius_points.append(tuple([dist,samples[x][2]]))
            if (len(radius_points) == n):
                radius_points.sort(key=lambda pt: pt[0])
                max_point = radius_points[n-1]
                max_dist = max_point[0]
        else:
            if (dist < max_dist):
                radius_points.remove(max_point)
                radius_points.append(tuple([dist,samples[x][2]]))
                radius_points.sort(key=lambda pt: pt[0])
                max_point = radius_points[n-1]
                max_dist = max_point[0]
    #weigh points using ratio to maximum distance
    for r in range(0,n):
        pt = radius_points[r]
        #radius_points[r] = tuple([max_dist/pt[0], pt[1]])
    for pt in radius_points:
        #classifier[pt[1]] = classifier[pt[1]] + pt[0]
        classifier[pt[1]] = classifier[pt[1]] + 1
    if (classifier[0] > classifier[1]):
        results.append(tuple([point[0],point[1],0]))
    else:
        results.append(tuple([point[0],point[1],1]))

    return

def KNNTwo(n, point,limit, samples):
    classifier = {0.0:0, 1.0:0}
    radius_points = []
    for x in range (0, limit):
        dist = euclideanDistance(point,samples[x])
        radius_points.append(tuple([dist,samples[x][2]]))
    radius_points.sort(key=lambda pt: pt[0])
    max_dist = radius_points[n-1][0]
    for x in range (0,n):
        pt = radius_points[x]
        prop_dist = max_dist/pt[0]
        classifier[pt[1]] = classifier[pt[1]] + 1
        #classifier[pt[1]] = classifier[pt[1]] + prop_dist
    return classifier

knn_errors = []
def tester(n,percent):
    global results
    results = []
    knn_results = []
    true_samples = len(data)
    num_samples = int(math.floor((1-percent)*true_samples))
    test_data = []
    for x in range (0, num_samples):
        test_data.append(data[x])
    #for x in range (num_samples+1,true_samples):
        #KNN(n,data[x],num_samples,test_data)
    for x in range (num_samples,true_samples):
        knn_results.append(KNNTwo(n,data[x],num_samples,test_data))

    error = 0.0
    for x in range (num_samples,true_samples):
        classifier = knn_results[x-num_samples]
        result = max(classifier, key=classifier.get)
        #print data[x][2],result
        if (result != data[x][2]):
            error+=1.0
        results.append([data[x][0], data[x][1], result])

    print results
    knn_errors.append(error/(true_samples-num_samples))

if __name__ == "__main__":

    readFile()
    for i in range(1,11):
        tester(i,0.2)
    print knn_errors
