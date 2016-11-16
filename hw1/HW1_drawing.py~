#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    #### YOUR CODE HERE ####
    import itertools

    handles = []
    radius = 0
    center = numpy.zeros(3)
    #iterate through all bodies in the environment, getting all the tables
    for body in env.GetBodies():
        if ('Table' in body.GetName()):
            #use the table's location and bounds to form the corner points
            tVec = body.GetTransform()[0:3,3]
            tBox = body.ComputeAABB().extents()
 
            z = tVec[2]
            pointsX = [[tVec[0]], [tBox[0], -tBox[0]]]
            pointsY = [[tVec[1]], [tBox[1], -tBox[1]]]

            points = []
            #corner points = sums of location and +/- extents in the XY axes
            for x in itertools.product(*pointsX):
                ptX = x[0] + x[1]
                for y in itertools.product(*pointsY):
                    ptY = y[0] + y[1]
                    points.append(tuple([ptX, ptY]))

            #exchange last and 2nd to the last point to create a box instead of an X figure
            temp = points[3]
            points[3] = points[2]
            points[2] = temp
            for i in range(0, len(points)):
                #specify points in clockwise order
                pt1 = points[i]
                pt2 = points[(i+1)%len(points)]
                #draw figure
                handles.append(env.drawlinestrip(points=array(((pt1[0],pt1[1],z),(pt2[0],pt2[1],z))),
                                                 linewidth=3.0,
                                                 colors=array(((1,0,0)))))

        #get the size of the room (to specify how far the circle plots will be)
        elif (body.GetName() == 'ProjectRoom'):
            radius = 1.4 * body.ComputeAABB().extents()[0]
            center = body.GetTransform()[0:3,3]

    #created a evenly partitioned angle array
    spaced = numpy.linspace(0, 2*numpy.pi, 35)
    for angle in spaced:
        #location based on trigonometric calculation, with repsect to the room center and calulcuated radius
        loc = numpy.array([center[0] + numpy.cos(angle)*radius,
                           center[1] + numpy.sin(angle)*radius,
                           center[2]])
        c_rad = radius/30
        handles.append(env.plot3(points=array((loc)),
                                   pointsize=c_rad,
                                   colors=array(((0,0,1))),
                                   drawstyle=1))
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
