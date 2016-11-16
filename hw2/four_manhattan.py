import heapq
import math
import sys
import time
import openravepy
import numpy as np

class Cell:
    config = []
    gscore = 0
    fscore = 0
    def set(self, config)

class Four_Manhattan:
    env = None
    robot = None
    goalconfig = []

    steplength = 0.15
    rotationlength = np.pi/4

    def init(self, e, r):
        self.env = e
        self.robot = r

    def astar(self, goalconfig):
        self.goalconfig = goalconfig

        #x-y theta from given goal configuration
        goal_x = goalconfig[0]
        goal_y = goalconfig[1]
        goal_o = goalconfig[2]
        goal_f = 0

        goalcell = [goal_x, goal_y, goal_o, goal_f]
        #goal z = same as robot z
        startconfig = self.robot.GetActiveDOFValues()
        startcell = [startconfig[0], startconfig[1], startconfig[2], 0] #4th element is the gscore

        openset =  []
        camefrom = {}

        heapq.heappush(openset, tuple([startcell, self.heuristic(startconfig, goalconfig)])) #2nd part of tuple = fscore
        openlist = [startconfig]

        #check if generated state is valid
        while (len(openset) > 0): #astar loop, modify later
            curcell = heapq.heappop(openset)

            closedset = []
            openlist.remove(curcell)
            curconfig = curcell[0]
            curscore = curconfig[3]

            #generate children
            self.robot.SetActiveDOFValues(curconfig);
            self.robot.GetController().SetDesired(self.robot.GetDOFValues());

            if curconfig == self.goalconfig:
                break

            nb = self.neighbors(curconfig)

            for n in nb:
                if n in closedset:
                    continue

                valid = True
                self.robot.SetActiveDOFValues(curconfig);
                for body in self.env.GetBodies():
                    if (body.GetName() != 'PR2' and self.env.CheckCollision(self.robot,body)):
                        valid = False
                        break

                if not valid:
                    continue

                #add valid cell to heapque
                tempscore = curscore + self.heuristic(curconfig, n)
                if n not in openlist or tempscore < curscore:
                    camefrom[n] = curconfig

                nscore = self.heuristic(n, self.goalconfig)
                heapq.heappush(openset, tuple([n, nscore]))

            self.robot.SetActiveDOFValues(curconfig);

    def heuristic(self, curconfig, newconfig):

        r1 = np.sqrt(np.power(curconfig[0],2) + np.power(curconfig[0],2))
        a1 = r1*np.cos(curconfig[2])
        b1 = r1*np.sin(curconfig[2])

        r2 = np.sqrt(np.power(newconfig[0],2) + np.power(newconfig[0],2))
        a2 = r2*np.cos(newconfig[2])
        b2 = r2*np.sin(newconfig[2])

        xt = newconfig[0] - curconfig[0]
        yt = newconfig[1] - curconfig[0]
        at = a2 - a1
        bt = b2 - b1

        return np.abs(xt) + np.abs(yt) + np.abs(at) + np.abs(bt)  

    def neighbors(self, cell):
        #generate 4 positional neighbors, each with 4 rotational neighbors
        n = []

        #generate positional neighbors
        xneighbors = -1
        yneighbors = -1
        for i in range(0, 4):
            x = cell[0]
            y = cell[1]
            o = cell[2]

            if (xneighbors < 2):
                x += (xneighbors * self.steplength)
                xneighbors += 2
            else:
                y += (yneighbors * self.steplength)
                yneighbors += 2

            #generate angular neighbors
            oinc = np.pi * 0.25
            for j in range(0,4):
                o += oinc
                newcell = [x, y, o]
                n.append(newcell)

        return n
     
#reference
#wall = env.GetKinBody('ProjectRoom')
#current_state = robot.GetActiveDOFValues()
#robot.SetActiveDOFValues([1.0  , current_state[1], current_state[2]]);
#robot.GetController().SetDesired(robot.GetDOFValues());