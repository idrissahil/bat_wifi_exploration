import random
import math
x_charge=5
y_charge=5
z_charge=5


class Node():
    def __init__(self, x, y, z, x_diff, y_diff,z_diff, parent_node = None):
        self.x = x
        self.y = y
        self.z = z
        self.parent_node = parent_node
        self.total_parents=0
        self.x_diff = x_diff
        self.y_diff = y_diff
        self.z_diff = z_diff

start_x=0
start_y=0
start_z=1



def rand_node():
    #print(goal_distance)
    x_min = -10
    y_min = -10
    z_min = -10

    x_max = 10
    y_max = 10
    z_max = 10
    x_rand = random.uniform(x_min, x_max)
    y_rand = random.uniform(y_min, y_max)
    z_rand = random.uniform(z_min, z_max)

    #if goal_distance%0.1 <0.01: #Boost the search towards the goal
        #print('boost')
    #    x_rand = random.uniform(car_1.xt-0.5, car_1.xt + 0.1)
    #    y_rand = random.uniform(car_1.yt-0.5, car_1.yt + 0.5)

    return x_rand, y_rand, z_rand

def find_closest_node(x_rand, y_rand, z_rand, node_list):
    dist_list = []
    for node in node_list:
        distance = math.sqrt(math.pow((node.x - x_rand), 2) + math.pow((node.y - y_rand), 2)+ math.pow((node.z - z_rand), 2))
        dist_list.append(distance)
    goal_distance = min(dist_list)
    closest_index = dist_list.index(min(dist_list))
    closest_node = node_list[closest_index]
    return closest_node, goal_distance, closest_index

def find_velocity(closest_node, x_rand, y_rand, z_rand):

    x_diff=x_rand-closest_node.x #meter per second speed
    y_diff=y_rand-closest_node.y
    z_diff=z_rand-closest_node.z

    return x_diff, y_diff, z_diff


def go_to_goal(near_x , near_y, near_z, x_diff, y_diff, z_diff):
    counter = 0
    curr_x=near_x
    curr_y=near_y
    curr_z=near_z
    collision = True
    while counter < 70:
        curr_x = curr_x + x_diff*0.0001  #Go to node for 70 ms
        curr_y = curr_y + y_diff*0.0001
        curr_z = curr_z + z_diff*0.0001
        counter = counter + 1
        #collision = Collision(curr_x, curr_y, obstacle_list)
        if collision==False:
            break
    return curr_x, curr_y, curr_z, collision, counter


def backtracking(Node_list, final_node):
    controls_x = []
    controls_y = []
    controls_z = []

    times = []
    index = len(Node_list)-1
    print(index)
    car_time=0
    for i in range(final_node.total_parents):
        counter=0
        while counter<70:
            true_node = Node_list[index]
            times.append(car_time)
            controls_x.append(true_node.x_diff)
            controls_y.append(true_node.y_diff)
            controls_z.append(true_node.z_diff)

            index = true_node.parent_node

            counter=counter+1
    times.append(car_time)
    #print('times',len(times))
    #print('controls',len(controls))
    #controls.reverse()

    return controls_x, controls_y,controls_z, times

start_node = Node(start_x, start_y, start_z, x_diff=0, y_diff=0, z_diff=0)
Node_List = [start_node]
goal_reach_distance = 2
goal_reached = False
while goal_reached == False:
    x_rand, y_rand, z_rand = rand_node()
    closest_node, goal_distance, closest_index = find_closest_node(x_rand, y_rand,z_rand, Node_List)
    x_diff, y_diff, z_diff = find_velocity(closest_node, x_rand, y_rand, z_rand)

    curr_x, curr_y, curr_z, collision, counter = go_to_goal( closest_node.x , closest_node.y, closest_node.z, x_diff, y_diff, z_diff)
    Suc_Node = Node(x=curr_x, y=curr_y,z=curr_z, parent_node=closest_index, x_diff=x_diff, y_diff=y_diff, z_diff=z_diff)
    #print(collision)
    if collision==True:
        Suc_Node.total_parents = 1 + closest_node.total_parents
        Node_List.append(Suc_Node)
        ignore1, goal_distance2, ignore2 = find_closest_node(x_charge, y_charge, z_charge, Node_List)
        #print(goal_distance2)
    if goal_distance2 <= goal_reach_distance:
        print("reached", goal_distance2)
        goal_reached = True

index = len(Node_List)-1
test_node=  Node_List[index]

controls_x, controls_y,controls_z, times = backtracking(Node_List, Suc_Node)

print(sum(controls_x))
