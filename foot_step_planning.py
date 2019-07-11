#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Foot step planning for a Humanoid robot in Openrave.
Thomas Tian (tianran@umich.edu), Feb. 2019
"""

import time
import openravepy
#### YOUR IMPORTS GO HERE ####
from foot_funLib import *
import heapq

#### END OF YOUR IMPORTS ####

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


def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'') 
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj




if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('foot.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    left_foot = env.GetKinBody('left_foot')
    right_foot = env.GetKinBody('right_foot')
    
    
    # path = None
    with env:
    #     # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
    #     robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goal_config = [2.6,-1.3,-pi/2]
        start = time.clock()
    #         #### YOUR CODE HERE ####
    #     # define the planner config

        start_config = [-2.6, -1.0, 0.0 ,-2.6, -1.3, 0.0]
       
   
        # print the planner information
        print'A* Path Planner for foot step planning'
        #print'Heuristic:', planner_config["heuristic_type"]
        #print'Connectivity:', planner_config["connectivity_type"]
        print'dx: 0.3 m'
        print'dy: 0.2 m'
        print'd_theta: 0.125pi rad'

    #     #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        path, handles = A_star_search(start_config, goal_config, env,left_foot,right_foot)
    #     #### Draw the X and Y components of the configurations explored by your algorithm
        handles = []
        handles.append(env.plot3(points=array((goal_config[0],goal_config[1],0.1)),
                                  pointsize=0.1,
                                   drawstyle = 1,
                                   colors=array(((0,1,0)))))
        for item in path:
            x_l = item[0]
            y_l = item[1]
            theta_l = item[2]
            z = 0.08
            l_table = 0.3
            w_table = 0.15
            # calculate the bounding box points
            left_box = array(((x_l-l_table/2*cos(theta_l)-w_table/2*sin(theta_l), y_l-l_table/2*sin(theta_l)+w_table/2*cos(theta_l),z),
                         (x_l-l_table/2*cos(theta_l)+w_table/2*sin(theta_l), y_l-l_table/2*sin(theta_l)-w_table/2*cos(theta_l),z),
                         (x_l+l_table/2*cos(theta_l)+w_table/2*sin(theta_l), y_l+l_table/2*sin(theta_l)-w_table/2*cos(theta_l),z),
                         (x_l+l_table/2*cos(theta_l)-w_table/2*sin(theta_l), y_l+l_table/2*sin(theta_l)+w_table/2*cos(theta_l),z),
                         (x_l-l_table/2*cos(theta_l)-w_table/2*sin(theta_l), y_l-l_table/2*sin(theta_l)+w_table/2*cos(theta_l),z)))
            handles.append(env.drawlinestrip(points=left_box,
                                               linewidth=3,colors=array(((1,0,0),(1,0,0),(1,0,0),(1,0,0),(1,0,0)))))
            x_l = item[3]
            y_l = item[4]
            theta_l = item[5]
            z = 0.08
            l_table = 0.3
            w_table = 0.15
            # calculate the bounding box points
            left_box = array(((x_l-l_table/2*cos(theta_l)-w_table/2*sin(theta_l), y_l-l_table/2*sin(theta_l)+w_table/2*cos(theta_l),z),
                         (x_l-l_table/2*cos(theta_l)+w_table/2*sin(theta_l), y_l-l_table/2*sin(theta_l)-w_table/2*cos(theta_l),z),
                         (x_l+l_table/2*cos(theta_l)+w_table/2*sin(theta_l), y_l+l_table/2*sin(theta_l)-w_table/2*cos(theta_l),z),
                         (x_l+l_table/2*cos(theta_l)-w_table/2*sin(theta_l), y_l+l_table/2*sin(theta_l)+w_table/2*cos(theta_l),z),
                         (x_l-l_table/2*cos(theta_l)-w_table/2*sin(theta_l), y_l-l_table/2*sin(theta_l)+w_table/2*cos(theta_l),z)))

            #table_box = array(table_box)

            handles.append(env.drawlinestrip(points=left_box,
                                               linewidth=3,colors=array(((0,0,1),(0,0,1),(0,0,1),(0,0,1),(0,0,1)))))

            

    #     #### END OF YOUR CODE ###
    end = time.clock()
    print "Time to find the path is : ", end - start, 's'
    #     # Now that you have computed a path, convert it to an openrave trajectory 
    # traj = ConvertPathToTrajectory(robot, path)

    # # Execute the trajectory on the robot.
    # if traj != None:
    #     robot.GetController().SetPath(traj)


    # waitrobot(robot)

    raw_input("Press enter to exit...")

