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
import string

inf = []
maze = None;
curr_cell = (0,0);
start_cell = (0,0);
goal_cell = (0,0);
evaluated = []
open_set = None
came_from = None
closed_set = []
g_score = {}
f_score = {}

x_limit = 0;
y_limit = 0;

def readFile():
    global  maze, goal_cell, start_cell, curr_cell, inf, x_limit, y_limit

    f = open('maze.txt', 'r')
    i = 0
    for line in f:
        inf.append(line.rstrip());
        i+=1

    maze = [[] for x in range(i)]

    for x in range(i):
        line = inf[x].split(' ')
        for y in range(len(line)):
            maze[x].append(line[y])
            if (line[y] == 'G'):
                goal_cell = (y,x)
            elif (line[y] == 'S'):
                start_cell = (y,x);
                curr_cell = start_cell;

    x_limit = len(maze)
    y_limit = len(maze[0])

def manhattan_distance((a,b), (c,d)):
    return abs(d-b) + abs(c-a);

def heuristic((a,b), (c,d)):
    return  manhattan_distance((a,b), (c,d))

def path(current):
    total_path = [current]
    while (current in came_from):
        current = came_from[current]
        if (current != None):
            total_path.append(current)
    return  total_path

def neighbors(cell):
    x = cell[0]
    y = cell[1]

    n = []
    if (x > 0):
        if (y > 0):
            n.append((x,y-1))
        n.append((x-1,y))
    else:
        if (y > 0):
            n.append((x,y-1))
    if (x < x_limit):
        if (y < y_limit):
            n.append((x,y+1))
        n.append((x+1,y))
    else:
        if (y < y_limit):
            n.append((x,y+1))
    return n

if __name__ == "__main__":

    readFile()
    open_set =  Queue.PriorityQueue()
    came_from = {}

    #t = time.clock()

    for x in range (0,len(maze)):
        for y in range (0,len(maze[0])):
            g_score.update({tuple([x,y]) : np.inf})
            came_from.update({tuple([x,y]) : None })

    g_score[start_cell] = 0
    f_score[start_cell] = g_score[start_cell] + heuristic(start_cell, goal_cell)
    open_set.put(start_cell, f_score[start_cell])
    open_list = [start_cell]

    ans = None

    while (open_set.qsize() > 0):
        curr_cell = open_set.get();
        open_list.remove(curr_cell)
        if (curr_cell == goal_cell):
            ans =  path(goal_cell)
            break

        closed_set.append(curr_cell)
        n = neighbors(curr_cell)
        #print n
        for cell in n:
            if (cell in closed_set or maze[cell[1]][cell[0]] == '1'):
                continue
            #print 'cell',cell,maze[cell[0]][cell[1]]
            #temp_g_score = g_score[curr_cell] + manhattan_distance(cell, goal_cell)
            temp_g_score = g_score[curr_cell] + 1
            if (cell not in open_list or temp_g_score < g_score[cell]):
                came_from[cell] = curr_cell
                #print cell,'came_from',curr_cell
                g_score[cell] = temp_g_score
                f_score[cell] = g_score[cell] + heuristic(cell, goal_cell)
                if (cell not in open_list):
                    open_set.put(cell, f_score[cell])
                    open_list.append(cell)

    if (ans != None):
        print 'Path found:',ans
        for i in range (0,x_limit):
            s = string.join(maze[i])
            #print s
            l = []
            j = 0;
            for c in s:
                if (c == ' '):
                    continue
                elif (c == '1'):
                    l.append('X')
                elif (c == 'S'):
                    l.append('S')
                elif (c == 'G'):
                    l.append('G')
                else:
                    if (ans.__contains__((j,i))):
                        #print (j,i)
                        l.append('O')
                    elif (closed_set.__contains__((j,i))):
                        l.append('-')
                    else:
                        l.append(' ')
                j += 1
            #print l
            l = string.join((l))
            print l
    else:
        print 'No path found from',start_cell,'to',goal_cell,'.'

    #print time.clock() - t
