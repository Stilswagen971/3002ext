#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from rbe3002_lab3.srv import path_selection
from rbe3002_lab4.srv import get_centroids, get_cspace, get_centroidsResponse

# My import issues : Solved
# Manhattan distance implementation (How to use? Using odom point?) : Solved
# How to connect services to exec node?  Solved
# Should we pull out C-Space from pathplanner.py (Have specific node for request map) : Solved
# How to tie in gmapping the way we want to? : Solved
# How are you ______? (insert SA name here) : Exhausted
# Should we add gmapping to our service? : Possible Solution make little nodes with only one job (Write 5 line tests per component)
# Where will the request map stuff go? : Solved
class Lab4:
    def __init__(self):
        rospy.init_node('lab4')
        #self.grid= rospy.Subscriber('/map', OccupancyGrid, self.frontier)
        #self.centroid_list = rospy.Publisher('/centroid_list', Path, queue_size=5)
        self.centroid = rospy.Service('centroid_list', get_centroids, self.collect_frontiers)
        self.largest_frontier = rospy.Publisher('/largest_frontier', GridCells, queue_size=5)
        self.zeros = rospy.Publisher('/zeros', GridCells, queue_size=5)
        print("lab 4 node ready")
    @staticmethod

    def index_to_grid(mapdata, index):
        """
        Returns the (x,y) coordinate pair to the given 1d index from the occupancy map
        :param index [int] The 1d index.
        :return  [int] The (x,y) pair.
        """
        #index = y * width + x
        width = mapdata.info.width
        index = float(index)
        width = float(width)
        x = index % width
        y = math.floor(index/width)
        x = int(x)
        y = int(y)
        return (x, y)

    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        res = mapdata.info.resolution
        x0 = mapdata.info.origin.position.x
        y0 = mapdata.info.origin.position.y
        world_x = (x + 0.5) * res + x0 
        world_y = (y + 0.5) * res + y0
        return Point(world_x, world_y, 0.0)

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        width = mapdata.info.width
        index = y * width + x
        return index

    @staticmethod
    def N8_frontier(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        neighbors8 = [(x-1, y), (x+1, y), (x, y+1), (x, y-1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)]
        n8 = []
        
        for i in range(len(neighbors8)):
            index = Lab4.grid_to_index(mapdata, neighbors8[i][0], neighbors8[i][1])
            #print("neighbor val", mapdata.data[index])
            if mapdata.data[index] == -1:
                #print("next")
                return True
        #print("returning false")
        return False

    @staticmethod
    def neighbors_of_8(frontier_point):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid..
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        x = frontier_point[0]
        y = frontier_point[1]
        neighbors8 = [(x-1, y), (x+1, y), (x, y+1), (x, y-1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1),
        (x-2, y), (x+2, y), (x, y+2), (x, y-2), (x+2, y+2), (x-2, y+2), (x+2, y-2), (x-2, y-2)]

        return neighbors8


    
    def dict_to_list(self, dictionary):
        # Coverts a dictionary into a list

        frontier_list = []

        for key, value in dictionary.items():
            frontier_list.append(value)
        return frontier_list

    def list_to_point_msg(self, frontier_centroids):
        centroids =[]
        if len(frontier_centroids) == 0:
            point = Point()
            point.x = 0.0
            point.y = 0.0
            point.z = 0.0
            centroids.append(point)
        else:

            for i in range(len(frontier_centroids)):
                point = Point()
                point.x = frontier_centroids[i][0]
                point.y = frontier_centroids[i][1]
                point.z = 0.0
                centroids.append(point)

        return centroids

    
    def group_frontier(self, frontier_dict, frontier_point):

        frontier_n8 = Lab4.neighbors_of_8(frontier_point)
        isAdded = False
        #frontier_dict = dict(frontier_dict)
        #neighbors of one of frontier points
        # if its not a neighbor create new key
        if (len(frontier_dict.keys()) == 0):
            frontier_dict[1] = [frontier_point]
        else:
            #compare n8 to each list
            #if match found stop, add frontier point
            #else make new key add frontier point
            for i, v in frontier_dict.items(): # i is key, v is value stored inside
                
                for a in v:
                    if a in frontier_n8 and not isAdded:
                        v.append(frontier_point)
                        isAdded = True
            if not isAdded:
                frontier_dict[len(frontier_dict) + 1] = [frontier_point]
                isAdded = False
        return frontier_dict





    # ask about global
    def collect_frontiers(self, msg):
        mapdata= msg.map
        data = mapdata.data
        zeros = []
        set_of_n8 = set()
        # find cells with value 0
        # do less checks, maybe add boolean for if it was checked already
        #isChecked = False
        checked = []
        # if not isChecked:

        # then find the ones with neighbors == -1
        for i in range(len(data)):
            if data[i]== 0:
                zero = Lab4.index_to_grid(mapdata, i)
                n8 = self.N8_frontier(mapdata, zero[0], zero[1])    
                if n8:
                    set_of_n8.add(zero)
                        #checked.append(n8)
                        #isChecked = True
        # vals=[]
        # zeros = list(set_of_n8)
        # for i in range(len(zeros)):
        #     index = Lab4.grid_to_index(mapdata, zeros[i][0], zeros[i][0])
        #     vals.append(data[i])
        # print("vals", vals)
            
        msg_grid_cells = GridCells()
        msg_grid_cells.header = mapdata.header

        set_of_n8 = list(set_of_n8)    
        for i in range(len(set_of_n8)):
            point = Lab4.grid_to_world(mapdata, set_of_n8[i][0], set_of_n8[i][1])
            msg_grid_cells.cells.append(point)
        #width and height
        res = mapdata.info.resolution
        msg_grid_cells.cell_width = res
        msg_grid_cells.cell_height = res
        self.zeros.publish(msg_grid_cells)


        if len(set_of_n8)==0:
            frontiers = []
            centroids = self.list_to_point_msg(frontiers)
            res = get_centroidsResponse(centroids=centroids)
            return res

        frontier_points = list(set_of_n8)
        frontiers = {}

        values=[]
        
        for i in range(len(frontier_points)):
            frontiers = self.group_frontier( frontiers, frontier_points[i])

        #print values
        
        # coverts to list
        frontier_list = self.dict_to_list(frontiers)

        length = len(frontier_list[0])
        # print("length", length)
        index = 0
        for i in range(len(frontier_list)):
            if len(frontier_list[i]) > length and len(frontier_list[i]) > 8:
                length = len(frontier_list[i])
                print("length", length)
                index = i
        largest_frontier = frontier_list[index]
            
        # GridCells msg for A* visited
        msg_grid_cells = GridCells()
        msg_grid_cells.header = mapdata.header
            
        for i in range(len(largest_frontier)):
            point = Lab4.grid_to_world(mapdata, largest_frontier[i][0], largest_frontier[i][1])
            msg_grid_cells.cells.append(point)
        #width and height
        res = mapdata.info.resolution
        msg_grid_cells.cell_width = res
        msg_grid_cells.cell_height = res
        self.largest_frontier.publish(msg_grid_cells)

        #print ("largest", largest_frontier)
        largest_frontier = self.list_to_point_msg(largest_frontier)
        res = get_centroidsResponse(centroids=largest_frontier)

        return res






    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        #self.frontier(1)
        print("all done!")
        rospy.spin()




if __name__ == '__main__':
    Lab4().run()