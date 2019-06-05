import time


class Node():
    def __init__(self, x, y, z, x_diff, y_diff, z_diff, total_distance, parent_node=None, x_ori=0, y_ori=0, z_ori=0, w_ori=0):
        self.x = x
        self.y = y
        self.z = z
        self.parent_node = parent_node
        self.total_parents = 0
        self.x_diff = x_diff
        self.y_diff = y_diff
        self.z_diff = z_diff
        self.total_distance = total_distance
        self.x_ori = x_ori
        self.y_ori = y_ori
        self.z_ori = z_ori
        self.w_ori = w_ori
Suc_Node = Node(x=1.5, y=2, z=3, parent_node=2, x_diff=1, y_diff=1,
                            z_diff=1, total_distance=0)
'''
def create_big_list(list):
    for i in range(1000):
        list.append(i)
    return list



def to_upper_case(s):
    return str(s).upper()

list1=[100,200]
def print_iterator(it):
    for x in it:
        print(x, end=' ')
    print('')  # for new line

# map() with string
map_iterator = map(to_upper_case, 'abc')
print(type(map_iterator))
print_iterator(map_iterator)

map_iterator2 = map(create_big_list, list1)
print(list(map_iterator2))
'''


items = [1, 2, 3, 4, 5]

def sqr(x, obstacle_list, Node):
    for i in range(len(obstacle_list)):
        print(Node.x)
        x=x*3 + Node.x
    return x





def Collision(x_list, obstacle_list):
    #start = time.time()
    collision = 0
    for i in range(len(obstacle_list)):
        dx = x_list[0] - obstacle_list[i].x
        dy = x_list[1] - obstacle_list[i].y
        dz = x_list[2] - obstacle_list[i].z

        #print("dx", dx)
        #print("dy", dy)
        #print("dz", dz)


        if abs(dx) < 0.25 and abs(dy) < 0.25 and abs(dz) < 0.25:
            print("abs collision check")
            collision = 1
        # dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)+ math.pow(dz, 2))
        # if dist <= 0.5:
        #    collision=True
        #    print("collision 111111111")
    if x_list[0] <= -40 or x > 40:
        print("2")
        collision = 1
    if x_list[1] <= -40 or y > 40:
        print("3")
        collision = 1
    if x_list[2] < -0.1 or z > 4.1:
        print("4")
        collision = 1

    #end = time.time()
    # print("Collision", end - start)
    return collision
x_list=[1,2,2]
obstacle_list = [[1,2,3],[1,2,3],[1,2,3],[1,2,3],[1,2,3],[1,2,3],[1,2,3],[1,2,3]]
#print(int(map(Collision, x_list, 1, 2, obstacle_list)))



start_map=time.time()
list(map(sqr, items, obstacle_list, Suc_Node))
end_map=time.time()
print("map time", end_map-start_map)

start_regular=time.time()
sqr(x_list,obstacle_list,Suc_Node)
end_regular=time.time()
print("regular time", end_regular-start_regular)

print("ratio", (end_regular-start_regular)/(end_map-start_map))

