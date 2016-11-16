#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def rodrigues(r):
    print 'RODRIGUES'
    def S(n):
        Sn = array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
        return Sn
    theta = linalg.norm(r)
    if theta > 1e-30:
        n = r/theta
        Sn = S(n)
        R = eye(3) + sin(theta)*Sn + (1-cos(theta))*dot(Sn,Sn)
        print 'n\n', n
        print 'Sn\n', Sn
        print 'sin(theta)*Sn\n',sin(theta)*Sn
        print '(1-cos(theta))*dot(Sn,Sn)\n', (1-cos(theta))*dot(Sn,Sn)
        print 'dot\n', dot(Sn,Sn)
        print 'sum\n', sin(theta)*Sn + (1-cos(theta))*dot(Sn,Sn)
    else:
        Sr = S(r)
        theta2 = theta**2
        R = eye(3) + (1-theta2/6.)*Sr + (.5-theta2/24.)*dot(Sr,Sr)
        print 'theta2\n', theta2
        print 'Sr\n', Sr
        print '(1-theta2/6.)*Sr\n', (1-theta2/6.)*Sr
        print '(.5-theta2/24.)*dot(Sr,Sr)\n', (.5-theta2/24.)*dot(Sr,Sr)
        print 'dot\n', dot(Sr,Sr)
        print 'sum\n', (1-theta2/6.)*Sr + (.5-theta2/24.)*dot(Sr,Sr)
    print "R\n", mat(R)
    return mat(R)

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    lmanip = robot.SetActiveManipulator("leftarm")
    rmanip = robot.SetActiveManipulator("rightarm")
    ### INITIALIZE YOUR PLUGIN HERE ###

    print lmanip.GetDirection()
    print rmanip.GetDirection()

    ### END INITIALIZING YOUR PLUGIN ###
   
    rodrigues([0, 0, 2.0944])

    n = 8
    m = 12

    inc1 = 2*numpy.pi / n
    inc2 = 2*numpy.pi / m

    rm3D = []
    order = []

    something = numpy.eye(4);
    print "something", something
    print "something", something[0:3, 3]
    print "something", something[0:4, 3]

    something[0:4, 3] = [1, 1, 1, 1]
    print "something", something

    print transpose(matrix([2, 2, 2]))
        
    # Rotate around Y (Approach from the front, back and sides)
    directionVector = lmanip.GetDirection()
    for axisIdx, axis in enumerate(directionVector):
        print "axisIdx, axis",axisIdx,axis
        if axis == 0:
            order.append(axisIdx)
        else:
            mainAxis = axisIdx


    for i in range(n):
        myList = [0,0,0]
        myList[order[0]] = i*inc1
        rot = rodrigues(myList)
        print "rot",rot
        rm3D.append(rot)

    myList = [0,0,0]
    myList[order[0]] = pi/2
    rot1 = rodrigues(myList)
    print "rot1", rot1
    for i in range(n):
        myList = [0,0,0]
        myList[order[1]] = i*inc1
        rot2 = rodrigues(myList)
        if(i != 0 and i != (n/2)):
            print "dot", dot(rot1,rot2)
            rm3D.append(dot(rot1,rot2))
    print "rm3D", rm3D

    if(m != 0):
        # Rotate aroundDir of all approach directions
        aroundDir = []
        for rIdx, r in enumerate(rm3D):
            # when i=0, the rotatin matrix is equal to r.
            # So we skip i=0.
            print "rIdx, r",rIdx,r
            for i in range(1,m):
                myList = [0,0,0]
                myList[mainAxis] = i*inc2
                rot1 = rodrigues(myList)
                aroundDir.append(dot(r,rot1))
        print "aroundDir", aroundDir

        temp = []
        for rIdx, r in enumerate(rm3D):
            temp.append(rm3D[rIdx])
            # There are 11 rotation matrices aroundZ
            # that's why range is between (i*0), and ((i+1)*11)
            for j in range((rIdx*(m-1)),(rIdx+1)*(m-1)):
                # print j
                temp.append(aroundDir[j])
        print "temp", temp


    print myList
    # tuck in the PR2's arms for driving
    #tuckarms(env,robot);
        

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

        goalconfig = [0.449,-0.201,0,0,0,0,0]

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
 
        ### END OF YOUR CODE ###
    #waitrobot(robot)

    raw_input("Press enter to exit...")

