#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped, TwistStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.msg import PositionTarget
import random
import math



rospy.init_node('rrt_charging_drone')

rrt_pub = rospy.Publisher('/mavros/setpoint_position/local2', PoseStamped, queue_size=1)
rrt_vis_pub = rospy.Publisher('visual_marker_rrt', PoseArray, queue_size=1)

rate = rospy.Rate(50)

ranges=[]
#success_node=[]
Node_List=[]
goal_node_list=[]
marks_list=[]


state_drone=1
index_rrt = 0

#x_charge= 15
#y_charge= -15
#z_charge=1


x_charge= 0
y_charge= 0
z_charge=1

class Node():
    def __init__(self, x, y, z, x_diff, y_diff,z_diff,total_distance, parent_node = None ):
        self.x = x
        self.y = y
        self.z = z
        self.parent_node = parent_node
        self.total_parents=0
        self.x_diff = x_diff
        self.y_diff = y_diff
        self.z_diff = z_diff
        self.total_distance = total_distance



boost_number=[0]
def rand_node(counter_boost, best_total_distance, min_distance, phi_rotation, x_half, y_half):
    c_best=best_total_distance
    c_min=min_distance
    if c_best==3000:
        #print(goal_distance) #regular rrt and first iteration
        x_min = -40
        y_min = -40
        z_min = -0.1

        x_max = 40
        y_max = 40
        z_max = 4
        x_rand = random.uniform(x_min, x_max)
        y_rand = random.uniform(y_min, y_max)
        z_rand = random.uniform(z_min, z_max)



    #informed rrt
    if c_best<3000:
        z_min = 0
        z_max = 2
        x_max_i=c_best
        #print(x_max_i)
        print("c_best", c_best, "c_min",c_min)
        #print("error value",(c_best*c_best - c_min*c_min) )
        y_max_i=math.sqrt(c_best*c_best - c_min*c_min)
        rho_rand=random.uniform(0, 1)
        phi_rand=random.uniform(0, 2*math.pi)
        x_rot = math.sqrt(rho_rand) * math.cos(phi_rand)
        y_rot = math.sqrt(rho_rand) * math.sin(phi_rand)

        x_rot = x_rot * x_max_i/2
        y_rot= y_rot * y_max_i/3

        x_rand=x_rot*math.cos(phi_rotation) - y_rot*math.sin(phi_rotation) + x_half
        y_rand = x_rot * math.sin(phi_rotation) + y_rot * math.cos(phi_rotation) + y_half
        z_rand = random.uniform(z_min, z_max)
        #print("x rand", x_rand)

    if counter_boost % 100 == 0:  # Boost the search towards the goal
        x_rand = x_charge
        y_rand = y_charge
        z_rand = z_charge
        # print("boost")
        boost_number[0] = boost_number[0] + 1



    return x_rand, y_rand, z_rand

def find_closest_node(x_rand, y_rand, z_rand, Node_List):
    dist_list = []
    for node in Node_List:
        distance = math.sqrt(math.pow((node.x - x_rand), 2) + math.pow((node.y - y_rand), 2)+ math.pow((node.z - z_rand), 2))
        dist_list.append(distance)
    goal_distance = min(dist_list)
    closest_index = dist_list.index(min(dist_list))
    closest_node = Node_List[closest_index]


    return closest_node, goal_distance, closest_index

def find_velocity(closest_node, x_rand, y_rand, z_rand):
    x_diff=x_rand-closest_node.x #meter per second speed
    y_diff=y_rand-closest_node.y
    z_diff=z_rand-closest_node.z
    speed_limit=1.5
    if x_diff >speed_limit:  #max speed set
        x_diff=speed_limit
    if x_diff <-speed_limit:
        x_diff=-speed_limit
    if y_diff >speed_limit:
        y_diff=speed_limit
    if y_diff <-speed_limit:
        y_diff=-speed_limit
    if z_diff >speed_limit:
        z_diff=speed_limit
    if z_diff <-speed_limit:
        z_diff=-speed_limit

    return x_diff, y_diff, z_diff


def round_of_rating(number, min_val):
    round_num=round(number * 2) / 2
    round_num= int((round_num-min_val)*2)


    return round_num

def index_collision_list(obstacle_list):
    min_x=-40
    min_y=-40
    min_z = -1

    max_x=40
    max_y=40
    max_z=40
    indexed_list=[]

    for ix in range((max_x-min_x)*2):
        curr_list_x=[]
        for iy in range((max_y-min_y)*2):
            curr_list_y = []
            for iz in range((max_z-min_z)*2):
                curr_list_z = []
                curr_list_y.append(curr_list_z)
            curr_list_x.append(curr_list_y)
        indexed_list.append(curr_list_x)





    for i in range(len(obstacle_list)):
        curr_x=obstacle_list[i].x
        curr_y=obstacle_list[i].y
        curr_z=obstacle_list[i].z
        rounded_x=round_of_rating(curr_x, min_x)
        rounded_y=round_of_rating(curr_y, min_y)
        rounded_z=round_of_rating(curr_z, min_z)

        indexed_list[rounded_x][rounded_y][rounded_z].append(obstacle_list[i])

    return indexed_list


def local_marks_list_finder(x_rounded, y_rounded, z_rounded, indexed_list):
    local_marks_list=[]
    local_marks_list.extend(indexed_list[x_rounded-1][y_rounded-1][z_rounded-1])
    local_marks_list.extend(indexed_list[x_rounded-1][y_rounded][z_rounded-1])
    local_marks_list.extend(indexed_list[x_rounded-1][y_rounded-1][z_rounded])
    local_marks_list.extend(indexed_list[x_rounded][y_rounded-1][z_rounded-1])
    local_marks_list.extend(indexed_list[x_rounded-1][y_rounded][z_rounded])
    local_marks_list.extend(indexed_list[x_rounded][y_rounded-1][z_rounded])
    local_marks_list.extend(indexed_list[x_rounded][y_rounded][z_rounded-1])

    local_marks_list.extend(indexed_list[x_rounded][y_rounded][z_rounded])
    local_marks_list.extend(indexed_list[x_rounded+1][y_rounded+1][z_rounded+1])
    local_marks_list.extend(indexed_list[x_rounded+1][y_rounded][z_rounded+1])
    local_marks_list.extend(indexed_list[x_rounded+1][y_rounded+1][z_rounded])
    local_marks_list.extend(indexed_list[x_rounded][y_rounded+1][z_rounded+1])
    local_marks_list.extend(indexed_list[x_rounded+1][y_rounded][z_rounded])
    local_marks_list.extend(indexed_list[x_rounded][y_rounded+1][z_rounded])
    local_marks_list.extend(indexed_list[x_rounded][y_rounded][z_rounded+1])

    return local_marks_list







def Collision(x, y, z, obstacle_list):
    collision = False
    for i in range(len(obstacle_list)):
        dx = x - obstacle_list[i].x
        dy = y - obstacle_list[i].y
        dz = z - obstacle_list[i].z

        if abs(dx)<0.30 and abs(dy)<0.30 and abs(dz)<0.2:
            #print("abs collision check")
            collision=True

        #dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)+ math.pow(dz, 2))
        #if dist <= 0.5:
        #    collision=True
        #    print("collision 111111111")
    if x <=-20  or x > 20:
        print("2")
        collision = True
    if y <= -20  or y > 20:
        print("3")
        collision = True
    if z < -0.1 or z > 4.1:
        #print("4")
        collision = True


    return collision

def go_to_goal(near_x , near_y, near_z, x_diff, y_diff, z_diff, marks_list):
    #start=time.time()
    curr_x=near_x
    curr_y=near_y
    curr_z=near_z
    distance_time=0.005
    step_num=100

    collision = False
    curr_x = curr_x + x_diff* distance_time*step_num  #Go to node for 100 *distance_time
    curr_y = curr_y + y_diff*distance_time*step_num
    curr_z = curr_z + z_diff*distance_time*step_num
    collision = Collision(curr_x, curr_y, curr_z, marks_list)
        #if collision==False:
            #break
    #end= time.time()
    #print("go_to_goal", end - start)
    return curr_x, curr_y, curr_z, collision

def go_to_goal2(near_x , near_y, near_z, x_diff, y_diff, z_diff, marks_list):
    #start=time.time()
    curr_x=near_x
    curr_y=near_y
    curr_z=near_z
    distance_time=0.01
    step_num=100
    counter=0
    collision = False
    while counter < step_num:
        curr_x = curr_x + x_diff* distance_time #Go to node for 100 *distance_time
        curr_y = curr_y + y_diff*distance_time
        curr_z = curr_z + z_diff*distance_time
        counter = counter + 1
        if counter%10==0:
            collision = Collision(curr_x, curr_y, curr_z, marks_list)
        if collision==True:
            curr_x = curr_x -int(counter/2)* x_diff * distance_time  # Go to node for 100 *distance_time
            curr_y = curr_y -int(counter/2)* y_diff * distance_time
            curr_z = curr_z -int(counter/2)* z_diff * distance_time
            collision = Collision(curr_x, curr_y, curr_z, marks_list)
            #print(collision)
            break
    #end= time.time()
    #print("go_to_goal2", end - start)
    return curr_x, curr_y, curr_z, collision
'''

def go_to_goal2(near_x , near_y, near_z, x_diff, y_diff, z_diff, marks_list):
    start=time.time()
    curr_x=near_x
    curr_y=near_y
    curr_z=near_z
    distance_time=0.002
    step_num=100
    counter=0
    collision = False
    curr_x = curr_x + step_num*x_diff* distance_time #Go to node for 100 *distance_time
    curr_y = curr_y + step_num*y_diff*distance_time
    curr_z = curr_z + step_num*z_diff*distance_time
    counter = counter + 1
    collision = Collision(curr_x, curr_y, curr_z, marks_list)
    if collision==True:
        curr_x = curr_x -int(step_num/2)* x_diff * distance_time  # Go to node for 100 *distance_time
        curr_y = curr_y -int(step_num/2)* y_diff * distance_time
        curr_z = curr_z -int(step_num/2)* z_diff * distance_time
        collision = Collision(curr_x, curr_y, curr_z, marks_list)
        #print(collision)
    end= time.time()
    print("go_to_goal2", end - start)
    return curr_x, curr_y, curr_z, collision
'''

def backtracking(Node_List, final_node):
    #start=time.time()
    controls_x = []
    controls_y = []
    controls_z = []
    x_drone =[]
    y_drone =[]
    z_drone =[]
    goal_node_list=[]

    times = []
    #index = len(Node_List)
    index=len(Node_List)-1
    print(index)
    car_time=0
    for i in range(final_node.total_parents):
        if index==None:
            break
        true_node = Node_List[index]
        goal_node_list.append(true_node)
        x_drone.append(true_node.x)
        y_drone.append(true_node.y)
        z_drone.append(true_node.z)

        controls_x.append(true_node.x_diff)
        controls_y.append(true_node.y_diff)
        controls_z.append(true_node.z_diff)
        index = true_node.parent_node
    x_drone.reverse()
    y_drone.reverse()
    z_drone.reverse()
    controls_x.reverse()
    controls_y.reverse()
    controls_z.reverse()
    goal_node_list.reverse()
    #end= time.time()
    #print("backtracking", end - start)
    return controls_x, controls_y,controls_z, x_drone, y_drone, z_drone, goal_node_list


def choose_parent(curr_x, curr_y, curr_z, Node_List, closest_index):
    #start=time.time()
    bounding_radius=1
    dist_list=[]
    dist_list_inside=[]
    inside_bound_list=[]
    parent_index=0
    best_dist=100000
    total_distance_constant=1
    goal_distance_curr = math.sqrt(math.pow((curr_x - x_charge), 2) + math.pow((curr_y - y_charge), 2) + math.pow((curr_z - z_charge), 2))

    #if goal_distance_curr<1:
    #    parent_index=closest_index

    for i in range(len(Node_List)):
        dx = curr_x - Node_List[i].x
        dy = curr_y - Node_List[i].y
        dz = curr_z - Node_List[i].z
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)+ math.pow(dz, 2))

        if dist<bounding_radius:
            inside_bound_list.append(i)
            dist_list_inside.append(dist)
            tot_dist=dist + Node_List[i].total_distance*total_distance_constant
            if i==0:
                tot_dist=tot_dist+1
            if tot_dist < best_dist:
                #print("lowered", i)
                parent_index = i
                best_dist = tot_dist
        dist_list.append(dist)
        if min(dist_list)>bounding_radius:
            #print("no nodes inside bouding area")
            parent_index=closest_index
    parent_node=Node_List[parent_index]
    return parent_index, parent_node, inside_bound_list, dist_list_inside

def rewire(inside_bound_list, Suc_node, Node_List, dist_list, local_marks_list):
    index_Suc = len(Node_List) - 1
    for i in range(len(inside_bound_list)):
        curr_node = Node_List[inside_bound_list[i]]
        curr_total_distance = curr_node.total_distance
        new_total_distance=Suc_node.total_distance + dist_list[i]
        if curr_total_distance>new_total_distance:
            x_diff, y_diff, z_diff = find_velocity(Suc_node, curr_node.x, curr_node.y, curr_node.z)
            curr_x, curr_y, curr_z, collision = go_to_goal2(Suc_node.x , Suc_node.y, Suc_node.z, x_diff, y_diff, z_diff, local_marks_list)
            if collision==False and abs(curr_x-curr_node.x)<0.1 and abs(curr_y-curr_node.y)<0.1 and abs(curr_z==curr_node.z) <0.1:
                curr_node.total_distance = new_total_distance
                curr_node.parent_node = index_Suc
                print("rewired 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    return Node_List



max_nodes_limit = 1000
def main_rrt(Node_List, start_x, start_y,start_z, marks_list, best_total_distance=3000, min_distance=0, phi_rotation=0):
    goal_reach_distance = 1
    if best_total_distance<2000:
        goal_reach_distance = 1
    goal_reached = False
    goal_distance2 = 50

    counter_boost = 0
    x_half=(x_charge-start_x)/2
    y_half = (y_charge - start_y)/2
    print("marks list", marks_list[1])

    indexed_list=index_collision_list(marks_list)

    #print("indexed_list", indexed_list[63][35][4])
    #print("indexed list", indexed_list)
    while goal_reached == False and len(Node_List) < max_nodes_limit:
        x_rand, y_rand, z_rand = rand_node(counter_boost, best_total_distance, min_distance, phi_rotation, x_half, y_half)
        closest_node, goal_distance, closest_index = find_closest_node(x_rand, y_rand, z_rand, Node_List)
        x_diff, y_diff, z_diff = find_velocity(closest_node, x_rand, y_rand, z_rand)
        x_round=round_of_rating(closest_node.x, -40)
        y_round=round_of_rating(closest_node.y, -40)
        z_round=round_of_rating(closest_node.z, -1)
        local_marks_list1=local_marks_list_finder(x_round, y_round, z_round, indexed_list)

        curr_x, curr_y, curr_z, collision = go_to_goal(closest_node.x, closest_node.y, closest_node.z, x_diff, y_diff, z_diff, local_marks_list1)
        parent_index, parent_node, inside_bound_list, dist_list_inside = choose_parent(curr_x, curr_y, curr_z, Node_List, closest_index)
        x_diff2, y_diff2, z_diff2 = find_velocity(parent_node, curr_x, curr_y, curr_z)

        x_round2=round_of_rating(parent_node.x, -40)
        y_round2=round_of_rating(parent_node.y, -40)
        z_round2=round_of_rating(parent_node.z, -1)
        local_marks_list2=local_marks_list_finder(x_round2, y_round2, z_round2, indexed_list)

        curr_x, curr_y, curr_z, collision = go_to_goal2(parent_node.x, parent_node.y, parent_node.z, x_diff2, y_diff2, z_diff2 , local_marks_list2)
        distance_travelled = math.sqrt(math.pow((curr_x - parent_node.x), 2) + math.pow((curr_y - parent_node.y), 2) + math.pow((curr_z - parent_node.z), 2))
        #print("distance travelled",distance_travelled)
        if collision == False:
            Suc_Node = Node(x=curr_x, y=curr_y, z=curr_z, parent_node=parent_index, x_diff=x_diff, y_diff=y_diff, z_diff=z_diff, total_distance=0)
            Suc_Node.total_parents = 1 + parent_node.total_parents
            Suc_Node.total_distance = distance_travelled + parent_node.total_distance
            Node_List.append(Suc_Node)
            Node_List = rewire(inside_bound_list, Suc_Node, Node_List, dist_list_inside, local_marks_list2)

            goal_distance_curr = math.sqrt(math.pow((x_charge - Suc_Node.x), 2) + math.pow((y_charge - Suc_Node.y), 2) + math.pow((z_charge - Suc_Node.z), 2))
            if goal_distance_curr<goal_distance2:
                goal_distance2=goal_distance_curr
            print("goal distance", goal_distance2, "new node x", Suc_Node.x)
            if goal_distance_curr <= goal_reach_distance and Suc_Node.total_distance<best_total_distance:
                print("reached !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", goal_distance_curr)
                goal_reached = True
        print("nodes", len(Node_List))
        counter_boost = counter_boost + 1
    controls_x, controls_y,controls_z, x_drone, y_drone, z_drone, goal_node_list = backtracking(Node_List, Suc_Node)

    print("number of nodes",len(Node_List))
    print("boost number", boost_number)
    print("x last node", Suc_Node.x, "ylast node", Suc_Node.y, "z last node", Suc_Node.z)
    #print(70*0.0001*sum(controls_x)) # controls given in 70*0.0001

    #print('x_start',x_drone[0], 'x_end', x_drone[-1])
    print("total distance",Suc_Node.total_distance)
    #plt.plot(x_drone, y_drone, 'ro')
    #plt.axis([-10, 10, -10, 10])
    #plt.show()
    return Suc_Node, Node_List, goal_node_list


def informed_rrt(start_x, start_y,start_z, marks_list):
    total_distance_list=[]
    total_node_list=[]
    min_distance =  math.sqrt(math.pow((x_charge - start_x), 2) + math.pow((y_charge - start_y), 2)) -1
    print("min distance", min_distance)
    phi_rotation = math.atan2(y_charge-start_y, x_charge - start_x)
    start_node = Node(start_x, start_y, start_z, x_diff=0, y_diff=0, z_diff=0, total_distance=0)
    Node_List = [start_node]
    for i in range(1): #10 iterations of rrt
        print("informed rrt iteration", i)
        print("node list length", len(Node_List))
        '''
        if i==0:
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y,start_z, marks_list)
            total_node_list.append(len(Node_List))
            total_distance_list.append(success_node.total_distance)
        '''
        if i==0:
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y,start_z, marks_list,min_distance+20, min_distance, phi_rotation )
            print("length of NOde list", len(Node_List))
            total_node_list.append(len(Node_List))
            total_distance_list.append(success_node.total_distance)
        if i>0:
            print("length of NOde list", len(Node_List))
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y, start_z, marks_list, success_node.total_distance, min_distance , phi_rotation)
            total_node_list.append(len(Node_List))
            total_distance_list.append(success_node.total_distance)
        if len(Node_List)==max_nodes_limit:
            break
    print("total node list",total_node_list )
    print("total distance list", total_distance_list)
    return success_node, Node_List, goal_node_list


def callback_gps(gps):
    global ranges
    global angle_min
    global state_drone
    global Node_List
    global goal_node_list
    global success_node
    global index_rrt
    global marks_list
    #print ("state drone", state_drone)
    if  not len(marks_list) ==0:
        rrt_poses = PoseArray()
        rrt_poses.header.stamp = rospy.Time.now()
        rrt_poses.header.frame_id = 'map'
        if state_drone==1:
            print("length marks list", len(marks_list))
            start=time.time()
            success_node, Node_List, goal_node_list = informed_rrt(gps.pose.position.x, gps.pose.position.y, gps.pose.position.z, marks_list )
            end=time.time()
            print("total time", end-start)
            state_drone = 2
        curr_rrt=PoseStamped()
        #print("index", index_rrt)
        #curr_rrt.pose.position.x=goal_node_list[index_rrt].x
        #curr_rrt.pose.position.y=goal_node_list[index_rrt].y
        #curr_rrt.pose.position.z=goal_node_list[index_rrt].z
        #curr_rrt.header.frame_id = "map"
        #rrt_pub.publish(curr_rrt)
        #distance_curr_rrt = math.sqrt(math.pow((gps.pose.position.x - goal_node_list[index_rrt].x), 2) + math.pow((gps.pose.position.y - goal_node_list[index_rrt].y), 2) + math.pow((gps.pose.position.z - goal_node_list[index_rrt].z), 2))
        #if distance_curr_rrt<0.5 and index_rrt<len(goal_node_list)-1:
        #    index_rrt=index_rrt+1

        for i in range(len(goal_node_list)):
            curr_point_rrt = Pose()
            curr_point_rrt.position.x = goal_node_list[i].x
            curr_point_rrt.position.y =  goal_node_list[i].y
            curr_point_rrt.position.z =  goal_node_list[i].z
            curr_point_rrt.orientation.x =  1
            rrt_poses.poses.append(curr_point_rrt)
        '''

        for i in range(len(Node_List)):
            curr_point_rrt = Pose()
            curr_point_rrt.position.x = Node_List[i].x
            curr_point_rrt.position.y = Node_List[i].y
            curr_point_rrt.position.z = Node_List[i].z
            curr_point_rrt.orientation.x = 1
            rrt_poses.poses.append(curr_point_rrt)
        '''




        rrt_vis_pub.publish(rrt_poses)

            #curr_point_rrt.orientation.z = -3.14 / 2
            #curr_point_rrt.orientation.x = 2
        #print ("state drone2", state_drone)
        #print("success node", Node_list[3].z)

def callback_markers(marks):
    global marks_list
    marks_unpacked=marks.markers[16].points
    marks_list = marks.markers[16].points
    #print("marks list", marks_list[2].x)

    #print("marks_unpacked first point", marks_unpacked[0].x)
    #print(marks_unpacked)


def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    vis_mark_sub = rospy.Subscriber('/aeplanner/occupied_cells_vis_array', MarkerArray, callback_markers)
    print('spinning')
    rospy.spin()

if __name__ == '__main__':
    main()
