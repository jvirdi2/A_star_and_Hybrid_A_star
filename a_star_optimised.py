import copy
import heapq as hq
import math
import matplotlib.pyplot as plt
import numpy as np

#from sets import Set

# car state = (x,y)
# state tuple (f,g,(x,y), [(x1,y1),(x2,y2)...])
# total cost f(n) = actual cost g(n) + heuristic cost h(n)
# obstacles = [(x,y), ...]
# min_x, max_x, min_y, max_y are the boundaries of the environment
class a_star:
    def __init__(self, min_x, max_x, min_y, max_y, \
            obstacle = [], resolution = 1, robot_size = 1) :
        ##TODO
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.obstacle = obstacle
        self.resolution = resolution
        self.robot_size = robot_size
        ####


    def euc_dist(self, position, target):
        val1 = np.sqrt(((position[0] - target[0]) ** 2) + ((position[1] - target[1]) ** 2))
        #val2 = abs(position[0]-target[0]) + abs(position[1]-target[1])
        return val1

    def costfunction(self, position, target):
        return 1
    

    # state: (total cost f, previous cost g, current position (x,y), \
    # previous motion id, path[(x1,y1),...])
    # start = (sx, sy)
    # end = (gx, gy)
    # sol_path = [(x1,y1),(x2,y2), ...]
    def Sort_Tuple(self,tup):  
    # reverse = None (Sorts in Ascending order)  
    # key is set to sort using second element of  
    # sublist lambda has been used  
        tup.sort(key = lambda x: x[1])  
        return tup

    def find_path(self, start, end):
        open_heap = [] # element of this list is like (cost,node)
        open_diction={} #  element of this list is like node: (cost,parent)
        
        visited_diction={} #  element of this list is like node: (cost,parent)
        

        
        
        obstacles = set(self.obstacle)
        cost_to_neighbour_from_start = 0
       

        hq.heappush(open_heap,((cost_to_neighbour_from_start + self.euc_dist(start, end),start)))
        
        open_diction[start]=(cost_to_neighbour_from_start + self.euc_dist(start, end),start)
        
        possible_neighbours=[(-1,-1),(-1,0),(-1,1),(0,1),(1,1),(1,0),(1,-1),(0,-1)]
        costs=[1.1, 1, 1.1, 0.5, 0.1, 0, 0.1, 0.5] # The goal always lies to the right
        # of the start node
        
        while len(open_heap) > 0:
            
            # choose the node that has minimum total cost for exploration
            
            chosen_node_cost =  open_heap[0]
   
            chosen_node=chosen_node_cost[1]
            chosen_cost=chosen_node_cost[0]
            
            
            
            visited_diction[chosen_node]=open_diction[chosen_node]
            
            if end in visited_diction:
                rev_final_path=[end] # reverse of final path
                node=end
                m=1
                while m==1:
                    contents=visited_diction[node]
                    parent_of_node=contents[1]
                    rev_final_path.append(parent_of_node)
                    node=parent_of_node
                    if node==start:
                        rev_final_path.append(start)
                        break
                final_path=[]
                for p in rev_final_path:
                    final_path.append(p)
                return final_path
            
            
            # explore this chosen element's neighbors 
            hq.heappop(open_heap) 
            for i in range(0,8):
                cost_to_neighbour_from_start =  chosen_cost-self.euc_dist(chosen_node, end)
                neigh_coord=possible_neighbours[i]             
                
                neighbour = (chosen_node[0]+neigh_coord[0],chosen_node[1]+neigh_coord[1])
                
                if ((neighbour not in obstacles) and \
                            (neighbour[0] >= self.min_x) and (neighbour[0] <= self.max_x) and \
                            (neighbour[1] >= self.min_y) and (neighbour[1] <= self.max_y)) :
                            
                            heurestic = self.euc_dist(neighbour,end)
                            cost_to_neighbour_from_start = 1+ cost_to_neighbour_from_start
                            total_cost = heurestic+cost_to_neighbour_from_start+costs[i]
                            
                            skip=0
                            #print(open_set_sorted)
                            # If the cost of going to this successor happens to be more
                            # than an already existing path in the open list to this successor,
                            # skip this successor
                            found_lower_cost_path_in_open=0 
                            if neighbour in open_diction:
                                
                                if total_cost>open_diction[neighbour][0]: 
                                    skip=1
                                    
                                elif neighbour in visited_diction:
                                    
                                    if total_cost>visited_diction[neighbour][0]:
                                        found_lower_cost_path_in_open=1
                                        
                           
                            if skip==0 and found_lower_cost_path_in_open==0:
                                
                                hq.heappush(open_heap,(total_cost,neighbour))
                                open_diction[neighbour]=(total_cost,chosen_node)
      
                            # If the successor is not in open list, we have to do one more check
                            # If this sucessor is already there in a visited path, and its cost is
                            # more than the path found in visited, ignore this. Otherwise, add
                            # to open list
                            
                           
                            
            #print(open_set_sorted)                      
             
        return []
                
                        
                        
                        
                        

def main():
    print(__file__ + " start!!")

    grid_size = 1  # [m]
    robot_size = 1.0  # [m]

    sx, sy = -10, -10   # originally -10, -10
    gx, gy = 10,10     # originally 10, 10
    obstacle = []
    for i in range(30):
        obstacle.append((i-15, -15))
        obstacle.append((i-14, 15))
        obstacle.append((-15, i-14))
        obstacle.append((15, i-15))
    
   
    
    for i in range(3):
        obstacle.append((0,i))
        obstacle.append((0,-i))

    # obstacle.append((-9, -9)) # not originally here

    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    simple_a_star = a_star(-15, 15, -15, 15, obstacle=obstacle, \
        resolution=grid_size, robot_size=robot_size)
    path = simple_a_star.find_path((sx,sy), (gx,gy))
    print (path)

    rx, ry = [], []
    for node in path:
        rx.append(node[0])
        ry.append(node[1])

    plt.plot(rx, ry, "-r")
    plt.show()


if __name__ == '__main__':
    main()
