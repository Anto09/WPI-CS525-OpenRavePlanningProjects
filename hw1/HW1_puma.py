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

    #get position and bounding box of pr2
    pr2Pos = robot.GetTransform()[0:3,3]
    pr2Box = robot.ComputeAABB()

    #load puma robot and specify its position
    env.Load('robots/puma.robot.xml')
    puma = env.GetRobots()[1]
    pumaBox = puma.ComputeAABB() #get bounding box of puma robot
    pMat = puma.GetTransform()  #get transformation matrix of puma
    xdist = pr2Box.extents()[0] + pumaBox.extents()[0] #set X offest from pr2 
    pMat[0:3,3] = robot.GetTransform()[0:3,3] + numpy.array([xdist, 0, 0]) #set new position of puma robot
    puma.SetTransform(pMat) #apply new transformation matrix

    #check if bases are colliding
    print env.CheckCollision(robot,puma) 

    #reach with the PR2 arm
    with robot:
        pi = numpy.pi
        jointnames_left = ['l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_wrist_flex_joint'] #specify joints to be used
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames_left]) #activate these joints for setting DOF Values
        robot.SetActiveDOFValues([-pi/2,pi/2, -pi/5, pi/4]) #set joint values
        robot.GetController().SetDesired(robot.GetDOFValues()) #tell controller to move arm to desired DOF values
    
    #ik-style code

    '''
    manip = robot.SetActiveManipulator('leftarm_torso')
    eeTransform = manip.GetEndEffectorTransform()

    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    with robot: # lock environment and save robot state
        #Tgoal = numpy.eye(4);
        #Tgoal[0:3,3] = puma.GetTransform()[0:3,3] 
        ikparam = IkParameterization(Tgoal,ikmodel.iktype) # build up the translation3d ik query
        sols = manip.FindIKSolutions(ikparam, IkFilterOptions.IgnoreEndEffectorCollisions) # get all solutions

    #h = env.plot3(Tgoal,10) # plot one point
    with robot: # save robot state
        raveLogInfo('%d solutions'%len(sols))
        for sol in sols: # go through every solution
            robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
            env.UpdatePublishedBodies() # allow viewer to update new robot
            time.sleep(10.0/len(sols))
    '''
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

