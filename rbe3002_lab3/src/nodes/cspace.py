#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path , Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue
from rbe3002_lab3.srv import path_selection
from rbe3002_lab4.srv import get_centroids, get_cspace


class Cspace:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("cspace")

        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        rospy.Service('/calc_cspace', get_cspace, self.calc_cspace)
        self.grid_cells = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=6)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic name, the message type is GridCells

        # robot intial position stuff
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        #rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # initialize variables
        self.pth = 0
        self.px = 0
        self.py = 0
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Cspace node ready")
        # hello




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
    def index_to_grid(mapdata, index):
        """
        Returns the (x,y) coordinate pair to the given 1d index from the occupancy map
        :param index [int] The 1d index.
        :return  [int] The (x,y) pair.
        """
        width = mapdata.info.width
        index = float(index)
        width = float(width)
        x = index % width
        y = math.floor(index/width)
        x = int(x)
        y = int(y)
        return (x, y)


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

    
    


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
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        res = mapdata.info.resolution
        x0 = mapdata.info.origin.position.x
        y0 = mapdata.info.origin.position.y
        grid_x = int((wp.x - x0) / res)
        grid_y = int((wp.y - y0) / res)
        return (grid_x, grid_y)



    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        height = mapdata.info.height
        width = mapdata.info.width
        index = Cspace.grid_to_index(mapdata, x, y)
        
        data = list(mapdata.data)
        if x <= (width-1) and x >= 0 and y >= 0 and y <= (height-1):
            if data[index] == 0: 
                return True
            else: 
                return False
        else: 
            return False
               
    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
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
            walkable = Cspace.is_cell_walkable(mapdata, neighbors8[i][0], neighbors8[i][1])
            if walkable:
                n8.append(neighbors8[i])
        return n8
 
    @staticmethod
    def find_obstacles(mapdata, data, n):
        """
        returns a list of the obstacles that contain the value 100-n
        :param mapdata [OccupancyGrid] The map information.
        :param n       [int]           n will be used to check the value of the cell
        :return        [[(int,int)]]   A list of obstacles with value 100-n.
        """
        print('finding obstacles')
        obstacles = []
        for i in range(len(data)):
            if data[i] == 100-n:
                obstacles.append(Cspace.index_to_grid(mapdata, i))
        print("obstacles found")
        return obstacles

    @staticmethod
    def mark_neighbors_8(mapdata, data, obstacles, n):
        """
        using the list of obstacles this function marks neighbors of 8 as non walkable.
        :param mapdata [OccupancyGrid] The map information.
        :param n       [int]           n will be used to check the value of the cell
        """
        print("marking neighbors")
        for i in range(len(obstacles)):
            N8 = Cspace.neighbors_of_8(mapdata, obstacles[i][0], obstacles[i][1])
            for k in range(len(N8)):
                data[Cspace.grid_to_index(mapdata, N8[k][0], N8[k][1])] = 99-n
        print('neighbors marked')
        return data
    



    
    def calc_cspace(self,msg):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        mapdata = msg.mapdata
        padding = 2
        rospy.loginfo("Calculating C-Space")
        # Go through each cell in the occupancy grid
        data = list(mapdata.data)
        n = 0
        done = False
        while not done:
                obstacle = Cspace.find_obstacles(mapdata,data, n)
                # Inflate the obstacles where necessary
                data = Cspace.mark_neighbors_8(mapdata, data, obstacle, n)
                n = n + 1
                if n == padding:
                    done = True
                
        # Create a GridCells message and publish it
        msg_grid_cells = GridCells()
        msg_grid_cells.header = mapdata.header
        # cells
        for i in range(len(data)):
            if data[i] >= 99-n and not data[i] == 100:
                (x, y) = Cspace.index_to_grid(mapdata, i)
                point = Cspace.grid_to_world(mapdata, x, y)
                msg_grid_cells.cells.append(point)
        # width and height
        h = mapdata.info.height
        w = mapdata.info.width
        res = mapdata.info.resolution
        msg_grid_cells.cell_width = res
        msg_grid_cells.cell_height = res
        self.grid_cells.publish(msg_grid_cells)
 
        # Return the C-space
        mapdata_cspace = OccupancyGrid()
        mapdata_cspace.header = mapdata.header
        mapdata_cspace.info = mapdata.info
        mapdata_cspace.data = data
        return mapdata_cspace
        

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        # map1 = self.request_map()
        # data = self.calc_cspace(map1, 1)
        # self.a_star(data, (2, 2), (10, 10))
        # print(data.data)
        print("all done!")
        rospy.spin()


if __name__ == '__main__':
    Cspace().run()


# <node pkg="rbe3002_lab3" type="path_planner.py" name="work_lab2" />
# launch file line
