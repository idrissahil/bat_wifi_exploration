#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Idris Sahil}
# {940510-9654}
# {sahil@kth.se}



import math
import random
import time


class Node():
    def __init__(self, x, y, theta=0, parent_node = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent_node = parent_node
        self.total_parents=0
        self.phi = 0


def go_to_goal(obstacle_list, phi, near_x , near_y, near_theta):
    counter = 0
    curr_x=near_x
    curr_y=near_y
    curr_theta = near_theta
    collision = True
    while counter < 70:

        counter = counter + 1
        collision = Collision(curr_x, curr_y, obstacle_list)
        if collision==False:
            break
    return curr_x, curr_y, curr_theta, collision, counter


def rand_node(goal_distance):
    #print(goal_distance)
    y_min = 0
    y_max = 10
    x_min = 0
    x_max = 20
    x_rand = random.uniform(x_min, x_max)
    y_rand = random.uniform(y_min, y_max)
    if goal_distance%0.1 <0.01: #Boost the search towards the goal
        #print('boost')

    return x_rand, y_rand

def find_angle_phi(closest_node, x_rand, y_rand):
    des_theta = math.atan2((y_rand - closest_node.y), (x_rand - closest_node.x))
    phi = math.atan2((des_theta - closest_node.theta), (0.01))

    if phi < -1 * math.pi / 4:
        phi = -1 * math.pi / 4
    elif phi > math.pi / 4:
        phi = math.pi / 4
    return phi

def backtracking(Node_list, final_node):
    controls = []
    times = []
    index = len(Node_list)-1
    car_time=0
    for i in range(final_node.total_parents):
        true_node = Node_list[index]
        times.append(car_time)
        controls.append(true_node.phi)
        index = true_node.parent_node
        car_time=car_time+70*car_1.dt
    times.append(car_time)
    #print('times',len(times))
    #print('controls',len(controls))
    controls.reverse()

    return controls, times


def find_closest_node(x_rand, y_rand, node_list):
    dist_list = []
    for node in node_list:
        distance = math.sqrt(math.pow((node.x - x_rand), 2) + math.pow((node.y - y_rand), 2))
        dist_list.append(distance)
    goal_distance = min(dist_list)
    closest_index = dist_list.index(min(dist_list))
    closest_node = node_list[closest_index]
    return closest_node, goal_distance, closest_index


def Collision(x, y, obstacle_list):
    collision = True
    for i in range(len(obstacle_list)):
        dx = x - obstacle_list[i][0]
        dy = y - obstacle_list[i][1]
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        if dist <= obstacle_list[i][2]+0.2:
            collision=False
        #print("1")
    if x <= 0.00001  or x > 19.8:
        #print("2")
        collision = False
    elif y <= 0.1  or y > 9.8:
        #print("3")
        collision = False
    return collision



start_time = time.time()

start_node = Node(car_1.x0, car_1.y0)
Node_List = [start_node]
goal_reach_distance = 0.75
goal_distance=30
goal_reached = False
while goal_reached == False:
    x_rand, y_rand = rand_node(goal_distance)
    closest_node, goal_distance, closest_index = find_closest_node(x_rand, y_rand, Node_List)
    phi = find_angle_phi(closest_node, x_rand, y_rand)

    curr_x, curr_y, curr_theta, collision, counter = go_to_goal(phi, closest_node.x, closest_node.y, closest_node.theta)
    Suc_Node = Node(x=curr_x, y=curr_y, parent_node=closest_index, theta=curr_theta)
    #print(collision)
    if collision==True:
        Suc_Node.total_parents = 1 + closest_node.total_parents
        Suc_Node.phi = phi
        Node_List.append(Suc_Node)
        ignore1, goal_distance2, ignore2 = find_closest_node(3, 3, Node_List)
    if goal_distance2 <= goal_reach_distance:
        print(goal_distance2)
        goal_reached = True

controls, times = backtracking(Node_List, Suc_Node)
print(controls[10])
end_time = time.time()
print('time', end_time - start_time)
