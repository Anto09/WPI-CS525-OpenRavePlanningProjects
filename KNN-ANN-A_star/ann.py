# -*- coding: utf-8 -*-
#!/usr/bin/python
__author__ = 'antoumali'

import random
import math
import sys
import time
import copy
import  Queue
import numpy as np
import  collections

data = []
results = []

def activation(x, p):
    return 1.0/(1.0 + np.exp(-x/p))

def aPrime(x, p):
    return activation(x,p)*(1.0-activation(x,p))

class NeuralNetwork:
    def __init__(self, layers):
        self.weights = []
        #randomize weights for each layer
        for i in range(1, len(layers) - 1):
            self.weights.append(2*np.random.random((layers[i-1] + 1, layers[i] + 1)) - 1)
        self.weights.append(2*np.random.random((layers[i] + 1, layers[i+1])) - 1)

    def backPropagationNormal(self, X, y, alpha):

        ones = np.atleast_2d(np.ones(X.shape[0]))
        X = np.concatenate((ones.T, X), axis=1)

        for i in range(len(X)):
            a = [X[i]]

            for l in range(len(self.weights)):
                    a.append(activation(np.dot(a[l], self.weights[l]), 1))

            error = y[i] - a[-1]
            deltas = [error * aPrime(a[-1],1)]

            for l in range(len(a) - 2, 0, -1):
                deltas.append(deltas[-1].dot(self.weights[l].T)*aPrime(a[l], 1))

            deltas.reverse()

            for i in range(len(self.weights)):
                layer = np.atleast_2d(a[i])
                delta = np.atleast_2d(deltas[i])
                self.weights[i] += alpha * layer.T.dot(delta)

    def backPropagationEpochs(self, X, y, alpha, epochs):

        X = np.concatenate((np.atleast_2d(np.ones(X.shape[0])).T, X), axis=1)

        for k in range(epochs):
            i = np.random.randint(X.shape[0])
            a = [X[i]]

            for l in range(len(self.weights)):
                    a.append(activation(np.dot(a[l], self.weights[l]), 1))

            error = y[i] - a[-1]
            deltas = [error * aPrime(a[-1],1)]

            for l in range(len(a) - 2, 0, -1):
                deltas.append(deltas[-1].dot(self.weights[l].T)*aPrime(a[l], 1))

            deltas.reverse()

            for i in range(len(self.weights)):
                layer = np.atleast_2d(a[i])
                delta = np.atleast_2d(deltas[i])
                self.weights[i] += alpha * layer.T.dot(delta)

    def classify(self, x):
        a = np.concatenate((np.ones(1).T, np.array(x)), axis=1)
        for l in range(0, len(self.weights)):
            a = activation(np.dot(a, self.weights[l]), 1)
        return a

def readFile():
    with open('data.txt') as openfileobject:
        for line in openfileobject:
            line = line.split(' ')
            data.append(tuple([float(line[0]), float(line[1]), float(line[2])]))


def tester(epochs, percent, hidden):
    global results
    results = []
    true_samples = len(data)
    num_samples = int(math.floor((1-percent)*true_samples))
    test_data = []

    inputs = []
    outputs = []
    queries = []

    network = NeuralNetwork([2,hidden,1])
    for x in range (0, true_samples):
        if (x < num_samples):
            test_data.append(data[x])
            inputs.append([data[x][0], data[x][1]])
            outputs.append(data[x][2])
        else:
            queries.append([data[x][0], data[x][1]])

    inputs = np.array(inputs)
    outputs = np.array(outputs)
    queries = np.array(queries)

    if (epochs == 0):
        network.backPropagationNormal(inputs, outputs, 0.2)
    else:
        network.backPropagationEpochs(inputs, outputs, 0.2, epochs)

    for e in queries:
        ans = np.round(network.classify(e))
        results.append([e[0], e[1], ans[0]])

    error = 0.0

    for x in range (num_samples,true_samples):
        index = x - num_samples
        if (results[index][2] != data[x][2]):
            error+=1.0
    #print error
    #print true_samples-num_samples
    #print error/(true_samples-num_samples)
    print 'Hidden neurons =',hidden
    print results
    return error/(true_samples-num_samples)

if __name__ == "__main__":

    readFile()
    error_results = []
    for i in range (2,11):
        error_results.append(tester(100000,0.9,i))
    print error_results
