import heapq
import math
import sys
import time
import openravepy
import numpy as np
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
        
class Four_Manhattan:
    env = None
    robot = None
    goalconfig = []
    goalstates = []
    startconfig = []

    steplength = 0.05
    rotationlength = np.pi/4

    #Truncate floats, from StackOverflow
    #Gets rid of precision issues with floating point numbers
    def truncate(f, n):
        '''Truncates/pads a float f to n decimal places without rounding'''
        s = '{}'.format(f)
        if 'e' in s or 'E' in s:
            return '{0:.{1}f}'.format(f, n)
        i, p, d = s.partition('.')
        return '.'.join([i, (d+'0'*n)[:n]])

    def init(self, e, r):
        self.env = e
        self.robot = r

    def checkgoal(self, config):
        epsilon = 0.001
        return (np.abs(config[0] - self.goalconfig[0]) < epsilon and
                np.abs(config[1] - self.goalconfig[1]) < epsilon)#and
                #np.abs(config[2] - self.goalconfig[2]) < epsilon)

    def checkgoalstates(self, config):
        for state in self.goalstates:
            if (self.checkgoal(config)):
                return True
        return False              

    def wrapangle(self):
        #wrap angles from [0, 2pi)
        #goal angle
        goalangle = self.goalconfig[2]
        signangle = np.sign(goalangle)
        absangle = np.abs(goalangle)

        while(absangle >= np.pi * 2):
            absangle -= np.pi * 2

        if (signangle < 0):
            absangle = np.abs(absangle) + np.pi

        goalangle = absangle
        self.goalconfig[2] = goalangle

        #start angle
        startangle = self.startconfig[2]
        signangle = np.sign(startangle)
        absangle = np.abs(startangle)

        while(absangle >= np.pi * 2):
            absangle -= np.pi * 2

        if (signangle < 0):
            absangle = np.abs(absangle) + np.pi

        startangle = absangle
        self.startconfig[2] = startangle
        #end wrap around section

    def generategoalstates(self):
        self.goalstates.append(self.goalconfig)

        xdiff = self.goalconfig[0] - self.startconfig[0]
        ydiff = self.goalconfig[1] - self.startconfig[1]
        angle = self.goalconfig[2]

        xsign = np.sign(xdiff)
        ysign = np.sign(ydiff)

        xdiv = np.abs(xdiff)/self.steplength
        ydiv = np.abs(ydiff)/self.steplength

        print np.floor(np.abs(xdiff)/self.steplength)
        print np.floor(np.abs(ydiff)/self.steplength)

        xrem = xdiv - np.floor(xdiv)
        yrem = ydiv - np.floor(ydiv)
        xmult = np.floor(xdiv)
        ymult = np.floor(ydiv)


        if (abs(xrem) > 0.00001):
            xleft = self.startconfig[0] + (xsign * xmult * self.steplength)
            xright = self.startconfig[0] + (xsign * (xmult + 1) * self.steplength)
            if (abs(yrem) > 0.00001):
                print 'X and Y'
                ybot = self.startconfig[1] + (ysign * ymult * self.steplength)
                ytop = self.startconfig[1] + (ysign * (ymult + 1) * self.steplength)

                self.goalstates.append([xleft, ybot, angle])
                self.goalstates.append([xleft, ytop, angle])
                self.goalstates.append([xright, ybot, angle])
                self.goalstates.append([xright, ytop, angle])
            else:
                print 'X only'
                self.goalstates.append([xleft, self.goalstates[1], angle])
                self.goalstates.append([xright, self.goalstates[1], angle])
        else:
            if (abs(yrem) > 0.00001):
                print 'Y only'
                ybot = self.startconfig[1] + (ysign * ymult * self.steplength)
                ytop = self.startconfig[1] + (ysign * (ymult + 1) * self.steplength)

                self.goalstates.append([self.goalstates[0], ybot, angle])
                self.goalstates.append([self.goalstates[0], ytop, angle])
            else:
                print 'No modification'
                self.goalstates.append([self.goalstates[1], self.goalstates[1], angle])

    def astar(self, goalconfig):
        handles = []
        self.goalconfig = goalconfig
        self.startconfig = self.robot.GetActiveDOFValues()

        self.wrapangle()

        startscore = self.heuristic(self.startconfig, self.goalconfig) #fscore
        self.generategoalstates()

        print 'startconfig', self.startconfig
        print 'goalconfig', self.goalconfig
        print 'goalstates', self.goalstates

        fscoreset = dict()
        gscoreset = dict()
        camefrom = dict()
        closedset = dict()
        openlist = dict()

        queue = []
        collisionlist = []
        validlist = []

        heapq.heappush(queue, tuple([startscore, self.startconfig])) #config as item, f-score as comparator
        fscoreset.update({tuple(self.startconfig): startscore}) #config as key, f-score as value
        gscoreset.update({tuple(self.startconfig): 0}) #config as key, g-score as 
        openlist.update({tuple(self.startconfig): 0}) #config as key, g-score as 

        ans = None
    
        iterations = 0
        #check if generated state is valid
        while (len(queue) > 0): #astar loop, modify later
            iterations += 1

            curcell = heapq.heappop(queue)
            curconfig = curcell[1]
            curscore = gscoreset[tuple(curconfig)]
            curf = fscoreset[tuple(curconfig)]

            del openlist[tuple(curconfig)]

            #print 'curconfig', curconfig #, curconfig in collisionlist
            #print 'cur g score',curscore
            #print 'cur f score',curf
           
            closedset.update({tuple(curconfig):curscore})

            #generate children
            self.robot.SetActiveDOFValues(curconfig);
            #self.robot.GetController().SetDesired(self.robot.GetDOFValues());

            loc = np.array([curconfig[0],curconfig[1], 0.05])
            handles.append(self.env.plot3(points=array((loc)), pointsize=3, colors=array(((0,1,0))), drawstyle=0))

            if (self.checkgoal(curconfig)):
                ans = curconfig
                break

            nb = self.neighbors(curconfig)

            for n in nb:
                #do not evaluate if closed already
                if n in closedset.keys():
                    continue

                #check validity by collision
                valid = True
                self.robot.SetActiveDOFValues(n);
                for body in self.env.GetBodies():
                    if (body.GetName() != 'PR2' and self.env.CheckCollision(self.robot,body)):
                        valid = False
                        collisionlist.append(n)
                        break

                if not valid:
                    loc = np.array([n[0],n[1], 0.05])
                    handles.append(self.env.plot3(points=array((loc)), pointsize=3, colors=array(((1,0,0))), drawstyle=0))

                    '''
                    inkeys = False
                    element = None
                    for i in range(0,len(queue)):
                        if np.array_equal(n, queue[i][1]):
                            inkeys = True
                            element = queue[i]
                            break
                    print 'invalid',n, n in validlist, inkeys
                    '''

                    continue
                #end collision check

                #add valid cell to heapque
                tempscore = self.heuristic(curconfig, n) + curscore
                notinkeys = n not in openlist.keys()
                if notinkeys or tempscore < curscore:
                    #print 'adding',n,n in collisionlist, n in closedset.keys()
                    validlist.append(n)
                    loc = np.array([n[0],n[1], 0.05])
                    handles.append(self.env.plot3(points=array((loc)), pointsize=3, colors=array(((0,0,1))), drawstyle=0))

                    if n not in camefrom.keys():
                        camefrom.update({n:curconfig})
                    else:
                        camefrom[n] = curconfig

                    cost = self.heuristic(n, goalconfig) + tempscore
                    if notinkeys:
                        gscoreset.update({n:tempscore})
                        fscoreset.update({n:cost})
                        openlist.update({n:tempscore})
                    else:
                        gscoreset[n] = tempscore
                        fscoreset[n] = cost
                        openlist[n] = tempscore
                    
                    #add to queue
                    heapq.heappush(queue, tuple([cost, n])) #config as item, f-score as comparator
                    
            self.robot.SetActiveDOFValues(curconfig);

        if ans is None:
            print 'Failure'
        else:
            print 'Success!'

        while (True):
            a = 1

        return

        if ans is None:
            print 'Failure'
            ans = self.robot.GetActiveDOFValues()
            totalpath = [tuple(ans)]

            current = tuple(ans)

            print current, fscoreset[current], camefrom[current]

            inkeys = current in camefrom.keys()

            while inkeys:
                hand = []
                loc = np.array([current[0],current[1], 0.05])
                hand.append(self.env.plot3(points=array((loc)), pointsize=0.1, colors=array(((1,0,0))), drawstyle=0))
                current = camefrom[current]
                totalpath.append(current)

                inkeys = current in camefrom.keys()

            while (True):
                a = 1

        else:
            totalpath = [ans]
            current = ans
            while current in camefrom:
                hand = []
                loc = np.array([current[0],current[1], 0.05])
                hand.append(self.env.plot3(points=array((loc)), pointsize=0.1, colors=array(((1,0,0))), drawstyle=0))
                current = camefrom[current]
                totalpath.append(current)
            
    def heuristic(self, curconfig, newconfig):

        a1 = np.cos(curconfig[2])
        b1 = np.sin(curconfig[2])

        a2 = np.cos(newconfig[2])
        b2 = np.sin(newconfig[2])

        xt = newconfig[0] - curconfig[0]
        yt = newconfig[1] - curconfig[0]
        at = a2 - a1
        bt = b2 - b1

        return np.abs(xt) + np.abs(yt) #+ np.abs(at) + np.abs(bt) 

    def neighbors(self, cell): #generates a 3-element list (config)
        #generate 4 positional neighbors, each with 4 rotational neighbors
        n = []

        #generate positional neighbors
        xneighbors = -1
        for i in range(0, 2):
            x = cell[0]
            o = cell[2]
                
            x += (xneighbors * self.steplength)
            xneighbors += 2

            yneighbors = -1
            for j in range(0, 2):
                y = cell[1]
                y += (yneighbors * self.steplength)
                yneighbors += 2

                newcell = [x, y, o]
                n.append(tuple(newcell))

                '''
                #generate angular neighbors
                o = cell[2]
                oinc = np.pi * 0.5
                for k in range(0,4):
                    o += oinc
                    if (np.abs(o) >= 2*np.pi):
                        o = (np.abs(o) - 2*np.pi)
                    newcell = [x, y, o]
                    n.append(tuple(newcell))
                '''
        return n
     
#reference
#wall = env.GetKinBody('ProjectRoom')
#current_state = robot.GetActiveDOFValues()
#robot.SetActiveDOFValues([1.0  , current_state[1], current_state[2]]);
#robot.GetController().SetDesired(robot.GetDOFValues());