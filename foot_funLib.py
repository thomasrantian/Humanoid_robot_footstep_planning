# -*- coding: utf-8 -*-
"""
Created on Tue Feb  5 13:11:52 2019

@author: tianr
"""

"""
Component functions for the planner.
Thomas Tian (tianran@umich.edu), Feb. 2019
"""

import numpy as np
from numpy import linalg as LA
from numpy import *
import heapq

"""
Node class: g is the moving cost, h is the heristic value, f = g + h
"""


class Node:
    x_l = None
    y_l = None
    theta_l = None

    x_r = None
    y_l = None
    theta_l = None
    g = None
    h = None
    f = None
    parentNode = None
    goal = None
    active_leg = None # indicating the configuration is caused by moving which lef, 'r' or 'l'

    reverse_flag = 0
    #flag = [[2.6,-1.3,-np.pi/2,2]]
    # flag = [[-0.1,-1.0,0.25*np.pi,1],[-0.05,1.0,0,2],[2.6,-1.3,-np.pi/2,3]]

    def __init__(self, pose):
        self.x_l = pose[0]
        self.y_l = pose[1]
        self.theta_l = pose[2]
        self.x_r = pose[3]
        self.y_r = pose[4]
        self.theta_r = pose[5]


    def __lt__(self, other):
        return self.f < other.f

        
        
    """
    Function to calculate the heuristic value: following the cost function in the paper
    """
    def update_h(self,goal):
        x_center = (self.x_r  + self.x_l) / 2
        y_center = (self.y_r + self.y_l) / 2
        theta_center = (self.theta_r + self.theta_l) / 2
        x_self = np.array([x_center, y_center])
        
        x_goal = np.array([goal[0], goal[1]])
        self.h =   LA.norm(x_self-x_goal)/0.3 + np.abs(theta_center - goal[2])/(0.125*np.pi)
        
            
    
    """
    Function to set g as zero 
    """
    def zero_g(self):
        self.g = 0   
        
    """
    Function to calculate the new g if goes to the node_next from the current
    """
    def get_g(self, node_next):
        x_self = np.array([self.x_l, self.y_l, self.theta_l,self.x_r, self.y_r, self.theta_r ])
        
        x_next = np.array([node_next.x_l, node_next.y_l, node_next.theta_l,node_next.x_r, node_next.y_r, node_next.theta_r ])
        return self.g + LA.norm(x_self-x_next)
        
            
    
    """
    Function to calculate and update the g value 
    """
    def update_g(self):
        x_self = np.array([self.x_l, self.y_l, self.theta_l,self.x_r, self.y_r, self.theta_r ])
        
        if self.parentNode == None:
            print "Updaing node with None parent!"
            return
        x_parent = np.array([self.parentNode.x_l, self.parentNode.y_l, self.parentNode.theta_l,self.parentNode.x_r, self.parentNode.y_r, self.parentNode.theta_r])
        self.g = self.parentNode.g + LA.norm(x_self-x_parent)
        
    
    """
    Function to calculate and update the f value 
    """   
    def update_f(self):
        reverse_cost = 0
        if self.reverse_flag:
            reverse_cost =0
        self.f = self.g + 1.5*self.h + reverse_cost

   
        
    """
    Function to explore the neighbors of the current node
    mode = 4 : four connectivity
    mode = 8: eight connectivity
    
    """
    def explore_neighbors(self):
        # store the [x_n, y_n, theta_n]
        sucessor_list = []

        if self.active_leg == 'l':
            # if the current config is due to moving l, I can only move my right legs 
            # explore the left foot neighbor configs in the right
            
            d_theta_set = [0, 0.125*np.pi, 0.125*np.pi]
            d_x_set = [0.25]
            d_y_set = [0.25, 0, -0.25]

            for x_id in range(len(d_x_set)):
                for y_id in range(len(d_y_set)):
                    for theta_id in range(len(d_theta_set)):
                        x_r_next = d_x_set[x_id]
                        y_r_next = d_y_set[y_id]
                        if y_r_next <= 0:
                            theta_r_next = self.theta_l
                        else:
                            theta_r_next = self.theta_l + d_theta_set[theta_id]
                        if theta_r_next > np.pi:
                            theta_r_next = theta_r_next - 2 *np.pi 
#                        if theta_r_next < 0:
#                            theta_r_next = theta_r_next + 2*np.pi
                        # covert the  x_l and y_l in to global frame
                        theta_rotate =  self.theta_l - np.pi/2

                        R = np.array([[np.cos(theta_rotate), -np.sin(theta_rotate), self.x_l],
                                        [np.sin(theta_rotate), np.cos(theta_rotate), self.y_l],
                                        [0., 0., 1]])
                        temp = np.dot(R,np.array([[x_r_next], [y_r_next], [1.]]))
                        reverse = 0
                        if y_r_next < 0:
                            reverse = 1
                        neighbor_pose = [round(self.x_l,1), round(self.y_l,1), round(self.theta_l,1), round(temp[0][0],1), round(temp[1][0],1), round(theta_r_next,1),reverse]
                        sucessor_list.append(neighbor_pose)
        if self.active_leg == 'r':
            d_theta_set = [0, 0.125*np.pi, -0.125*np.pi]
            d_x_set = [-0.25]
            d_y_set = [0.25, 0, -0.25]

            for x_id in range(len(d_x_set)):
                for y_id in range(len(d_y_set)):
                    for theta_id in range(len(d_theta_set)):
                        x_l_next = d_x_set[x_id]
                        y_l_next = d_y_set[y_id]
                        if y_l_next <= 0:
                            theta_l_next = self.theta_r
                        else:
                            theta_l_next = self.theta_r + d_theta_set[theta_id]
                        if theta_l_next > np.pi:
                            theta_l_next = theta_l_next - 2*np.pi
#                        if theta_l_next < 0:
#                            theta_l_next = theta_l_next + 2*np.pi
                        # covert the  x_l and y_l in to global frame
                        theta_rotate =  self.theta_r - np.pi/2

                        R = np.array([[np.cos(theta_rotate), -np.sin(theta_rotate), self.x_r],
                                        [np.sin(theta_rotate), np.cos(theta_rotate), self.y_r],
                                        [0., 0., 1]])
                        temp = np.dot(R,np.array([[x_l_next], [y_l_next], [1.]]))
                        reverse = 0
                        if y_l_next < 0:
                            reverse = 1
                        neighbor_pose = [round(temp[0][0],1), round(temp[1][0],1), round(theta_l_next,1),round(self.x_r,1),round(self.y_r,1),round(self.theta_r,1),reverse]
                        sucessor_list.append(neighbor_pose)
        
        return sucessor_list

"""
A_start searching function
"""
def A_star_search(start, goal, env, left_foot, right_foot):
    # build the PQ
   
    start_node = Node(start)
    start_node.zero_g()
    start_node.update_h(goal)
    start_node.update_f()
    start_node.active_leg = 'r'

    openSet = []   # use heapq to matain the order 
    openSet_dummy = {} # openSet in dictionary format, map the x,y,theta to node reference
    closeSet = {} # closeSet in dictionary format, map the x,y,theta to node reference
    path = []
    handles = []
    heapq.heappush(openSet,start_node)
    openSet_dummy[(start_node.x_l, start_node.y_l, start_node.theta_l,start_node.x_r, start_node.y_r, start_node.theta_r )] = start_node
    start_node.active_leg = 'r'
    while len(openSet) > 0:
        # pop the top node from the pq (the order is matained in this function)
        current_node = heapq.heappop(openSet)
        #print current_node.goal
        # delete the node from the openset_dummy
        #del openSet_dummy[(current_node.x_l, current_node.y_l, current_node.theta_l,start_node.x_r, start_node.y_r, start_node.theta_r)] 
        # check if the goal is reached
        if (np.abs(current_node.x_l- goal[0]) < 0.1 and np.abs(current_node.y_l- goal[1]) < 0.1 and np.abs(current_node.theta_l- goal[2]) < 0.2) or (np.abs(current_node.x_r- goal[0]) < 0.1 and np.abs(current_node.y_r- goal[1]) < 0.1 and np.abs(current_node.theta_r- goal[2]) < 0.2):
            current_node_copy = current_node
            while current_node_copy != None:
                path.append([current_node_copy.x_l, current_node_copy.y_l, current_node_copy.theta_l,current_node_copy.x_r, current_node_copy.y_r, current_node_copy.theta_r])
                current_node_copy = current_node_copy.parentNode
            
            print "Goal Found! Constructing path!"
            print "The path cost is ", round(current_node.g,1)
            left_temp_pose = array([[cos(current_node.theta_l),-sin(current_node.theta_l),0,current_node.x_l],
                     [sin(current_node.theta_l),cos(current_node.theta_l),0,current_node.y_l],
                     [0,0,1,0.08],
                     [0,0,0,1]])

            right_temp_pose = array([[cos(current_node.theta_r),-sin(current_node.theta_r),0,current_node.x_r],
                     [sin(current_node.theta_r),cos(current_node.theta_r),0,current_node.y_r],
                     [0,0,1,0.08],
                     [0,0,0,1]])
            left_foot.SetTransform(left_temp_pose)
            right_foot.SetTransform(right_temp_pose)
           
                
            return path[::-1], handles
           
        # put the popped node in the cloest set
        closeSet[(current_node.x_l, current_node.y_l, current_node.theta_l,current_node.x_r, current_node.y_r, current_node.theta_r)] = current_node
        
        # check the sucessors
        sucessor_list = current_node.explore_neighbors()
        #print sucessor_list
        for sucessor_pose in sucessor_list:
            # if the sucessor is already in the closeSet
            if (sucessor_pose[0],sucessor_pose[1], sucessor_pose[2],sucessor_pose[3],sucessor_pose[4], sucessor_pose[5]) in closeSet:
                continue
            # set the right foot and left foot to check collition

            left_temp_pose = array([[cos(sucessor_pose[2]),-sin(sucessor_pose[2]),0,sucessor_pose[0]],
                     [sin(sucessor_pose[2]),cos(sucessor_pose[2]),0,sucessor_pose[1]],
                     [0,0,1,0.08],
                     [0,0,0,1]])

            right_temp_pose = array([[cos(sucessor_pose[5]),-sin(sucessor_pose[5]),0,sucessor_pose[3]],
                     [sin(sucessor_pose[5]),cos(sucessor_pose[5]),0,sucessor_pose[4]],
                     [0,0,1,0.08],
                     [0,0,0,1]])
            left_foot.SetTransform(left_temp_pose)
            right_foot.SetTransform(right_temp_pose)
            
            if env.CheckCollision(left_foot) or env.CheckCollision(right_foot):
                handles = []
                x_l = sucessor_pose[0]
                y_l = sucessor_pose[1]
                theta_l = sucessor_pose[2]
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
                                                   linewidth=1,colors=array(((0,0,1),(0,0,1),(0,0,1),(0,0,1),(0,0,1)))))

                x_l = sucessor_pose[3]
                y_l = sucessor_pose[4]
                theta_l = sucessor_pose[5]
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
                                                   linewidth=1,colors=array(((0,0,1),(0,0,1),(0,0,1),(0,0,1),(0,0,1)))))
                continue
            
            # if the sucessor is in the openSet
            if (sucessor_pose[0],sucessor_pose[1], sucessor_pose[2],sucessor_pose[3],sucessor_pose[4], sucessor_pose[5]) in openSet_dummy:
                sucessor_node = openSet_dummy[(sucessor_pose[0],sucessor_pose[1], sucessor_pose[2],sucessor_pose[3],sucessor_pose[4], sucessor_pose[5])]
                # if the moving cost can be lower by reaching from the current
                if sucessor_node.g > current_node.get_g(sucessor_node):
                    sucessor_node.parentNode = current_node
                    sucessor_node.update_g()
                    sucessor_node.update_f()
                    # heapify
                    heapq.heapify(openSet)
                    #print'ddddddddddddddd'
            else:
                handles = []
                x_l = sucessor_pose[0]
                y_l = sucessor_pose[1]
                theta_l = sucessor_pose[2]
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
                                                   linewidth=1,colors=array(((1,0,0),(1,0,0),(1,0,0),(1,0,0),(1,0,0)))))
                
               
                x_l = sucessor_pose[3]
                y_l = sucessor_pose[4]
                theta_l = sucessor_pose[5]
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
                                                   linewidth=1,colors=array(((0,0,1),(0,0,1),(0,0,1),(0,0,1),(0,0,1)))))

                temp_node = Node(sucessor_pose[0:6])
                if sucessor_pose[6] == 1:
                    temp_node.reverse_flag = 1
                temp_node.parentNode = current_node
                if current_node.active_leg == 'r':
                    temp_node.active_leg = 'l'
                if current_node.active_leg == 'l':
                    temp_node.active_leg = 'r'

                temp_node.update_g()
                temp_node.update_h(goal)
                temp_node.update_f()
                heapq.heappush(openSet, temp_node)
                openSet_dummy[(temp_node.x_l,temp_node.y_l, temp_node.theta_l,temp_node.x_r,temp_node.y_r, temp_node.theta_r)] = temp_node
