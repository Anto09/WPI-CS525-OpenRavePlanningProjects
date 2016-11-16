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

    #original pose: (-3.4, -1.4, 0.05)
    #moved pos: (-0.00287, -1.33715, 0.05)

    #modify transformation of the robot
    rMat = robot.GetTransform();
    rMat[0:3,3] = numpy.array([-0.00287, -1.33715, 0.05])
    robot.SetTransform(rMat)
    wall = env.GetKinBody('ProjectRoom')

    #check if robot is touching the wall (should return false)
    print env.CheckCollision(robot,wall)

    #specify joint names
    jointnames = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 
    'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_l_finger_joint', 'r_gripper_motor_slider_joint', 
    'r_gripper_motor_screw_joint', 'r_gripper_joint']
       

    with env:
        #extend arm of the robot
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        angles = []
        for name in jointnames:
            if (name == 'r_shoulder_lift_joint' or name == 'r_elbow_flex_joint' or name == 'r_wrist_flex_joint'): #only 3 joints needed to fully extend the arm
                angles.append(0)
            else:
                angles.append(robot.GetJoint(name).GetValues()[0])
        robot.SetActiveDOFValues(angles);

        #should print true
        print env.CheckCollision(robot,wall)
 
        #move robot to spot and extend arm
        robot.GetController().SetDesired(robot.GetDOFValues());
    #wait so that robot reaches desired config
    waitrobot(robot)
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
