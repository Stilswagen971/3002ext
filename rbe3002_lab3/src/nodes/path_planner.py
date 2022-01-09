#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path , Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue
from rbe3002_lab3.srv import path_selection
from rbe3002_lab4.srv import get_centroids, get_cspace
from tf.transformations import euler_from_quaternion


class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.path = rospy.Service('path_plan', GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        #self.grid_cells = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=6)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic name, the message type is GridCells
        #fix this subscriber
        self.centroid_list = rospy.Service('/frontier_path', path_selection, self.plan_path_frontier)
        self.A_Star = rospy.Publisher('/astar', GridCells, queue_size=5)
        self.visited = rospy.Publisher('/visited', GridCells, queue_size=5)
        self.goal = rospy.Publisher('/goal_start', GridCells, queue_size=5)
        self.PATH = rospy.Publisher('/path_to_follow', Path, queue_size=5)
        ## Initialize the request counter
        # TODO
        # robot intial position stuff
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # initialize variables
        self.pth = 0
        self.px = 0
        self.py = 0
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")
        # hello


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw


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



    def point_to_tuple(self, largest_frontier):
        centroids = []
        for i in range(len(largest_frontier)):
            x = largest_frontier[i].x
            y = largest_frontier[i].y
            cord = (x,y)
            centroids.append(cord)
        return centroids
    
    def best_point(self, largest_frontier):
        index = int(math.floor(len(largest_frontier)/2))
        #index= len(frontier)-1
        for i in range(len(largest_frontier)):
            centroid = largest_frontier[index]
        return centroid



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
    # euclidena (smaller than manhattan) manhattan (May go though wall)
    # Use A* to solve possibly going through wall
    # Crude solution: Sort neighbors by angle; start from starting cell (direction vecctor), take closet cell by orientationn (maybe cw/ccw) Dot Product sort then for loop
    # list of cells, sort by angle, now for loop over sorted list
    


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
        #rospy.sleep(0.5)
        grid_x = int((wp.x - x0) / res)
        grid_y = int((wp.y - y0) / res)
        return (grid_x, grid_y)


        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        rbe3002_lab4/srv/get_cspace.srv
        pose.header = mapdata.header
        point = PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1])
        pose.pose.position = point
        Path.append(pose)
        return Path

    

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
        index = PathPlanner.grid_to_index(mapdata, x, y)
        
        data = list(mapdata.data)
        if x <= (width-1) and x >= 0 and y >= 0 and y <= (height-1):
            if data[index] == 0: 
                return True
            else: 
                return False
        else: 
            return False
               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """

        neighbors4 = [(x-1,y), (x+1,y), (x,y+1), (x,y-1)]
        n4 = []
        for i in range(len(neighbors4)):
            walkable = PathPlanner.is_cell_walkable(mapdata, neighbors4[i][0], neighbors4[i][1])
            if walkable:
                n4.append(neighbors4[i])
        return neighbors4
    
    
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
            walkable = PathPlanner.is_cell_walkable(mapdata, neighbors8[i][0], neighbors8[i][1])
            if walkable:
                n8.append(neighbors8[i])
        return n8
   



    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        #rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        """
        Creates an A* Algorithm
        """
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        visited = []
        path = []
        path.append(goal)

        while not frontier.empty():
            rospy.sleep(0.075)
            current = frontier.get()
            #msg_path = GridCells()
            #self.PATH.publish(msg_path)

            if current == goal:
                print("goal found in A*")
                x = came_from[goal]
                
                #print("came from  outside=", x)
                # path.append(x)
                while came_from[x] != start:
                    path.insert(0, x)
                    x = came_from[x]
                    #print ('came from inside =', x)
                path.insert(0, x)    

                # GridCells message for A* path
                msg_path = Path()
                msg_path.header = mapdata.header
                for i in range(len(path)):
                    pose_stamped = PoseStamped()
                    point = PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1])
                    pose_stamped.pose.position= point
                    msg_path.poses.append(pose_stamped)
                self.PATH.publish(msg_path)
                print ("path published")
                break

            for next in PathPlanner.neighbors_of_8(mapdata, current[0], current[1]):
                new_cost = cost_so_far[current] + self.euclidean_distance(current[0], current[1], next[0], next[1])
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.euclidean_distance(goal[0], goal[1], next[0], next[1])
                    frontier.put(next, priority)
                    came_from[next] = current
            visited.append(current)

            #self.gridcells_msg_publisher(mapdata, frontier.get_queue(), A_Star)
            #GridCells msg for A* frontier
            msg_grid_cells = GridCells()
            msg_grid_cells.header = mapdata.header
            f = frontier.get_queue()
            for i in range(len(f)):
                point = PathPlanner.grid_to_world(mapdata, f[i][1][0], f[i][1][1])
                msg_grid_cells.cells.append(point)
            #width and height
            res = mapdata.info.resolution
            msg_grid_cells.cell_width = res
            msg_grid_cells.cell_height = res
            self.A_Star.publish(msg_grid_cells)

            # GridCells msg for A* visited
            msg_grid_cells = GridCells()
            msg_grid_cells.header = mapdata.header
            f = visited
            for i in range(len(f)):
                point = PathPlanner.grid_to_world(mapdata, f[i][0], f[i][1])
                msg_grid_cells.cells.append(point)
            # width and height
            res = mapdata.info.resolution
            msg_grid_cells.cell_width = res
            msg_grid_cells.cell_height = res
            self.visited.publish(msg_grid_cells)

        # graph_cost is value of index
        # goal, next
        # heuristic is euclidean distance between goal and next
        return path

    def gridcells_msg_publisher(self, mapdata, f, msg):
        # GridCells msg for A* frontier
            msg_grid_cells = GridCells()
            msg_grid_cells.header = mapdata.header
            for i in range(len(f)):
                point = PathPlanner.grid_to_world(mapdata, f[i][1][0], f[i][1][1])
                msg_grid_cells.cells.append(point)
            # width and height
            res = mapdata.info.resolution
            msg_grid_cells.cell_width = res
            msg_grid_cells.cell_height = res
            self.msg.publish(msg_grid_cells)

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        opt_path = []
        opt_path.append(path[0])
        waypoints_skipped= 0
        for i in range(len(path)):
            x = path[i][0]
            y = path[i][1]
            if (i>=1):
                xprev = path[i-1][0]
                yprev = path[i-1][1]
                if not (x == xprev or y == yprev ) or i == (len(path)-1) or waypoints_skipped >= 10:
                    opt_path.append(path[i])
                    waypoints_skipped = 0
                else:
                    waypoints_skipped += 1
       
            
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        return opt_path

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        path_msg = Path()
        path_msg.header = mapdata.header
        poses = []
        for i in range(len(path)):
            point = PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1])
            pose = PoseStamped()
            pose.pose.position = point
            poses.append(pose)
        path_msg.poses = poses

        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        return path_msg

    def plan_path_frontier(self, msg):
        #print(msg)
        largest_frontier = msg.centroids
        mapdata = msg.mapdata
        #print(mapdata)
        largest_frontier_list = self.point_to_tuple(largest_frontier)
        goal = self.best_point(largest_frontier_list)
        index = PathPlanner.grid_to_index(mapdata, goal[0], goal[1])
        # print ("goal value", mapdata.data[int(index)])
        print("goal", goal)
        start = Point()
        start.x = self.px
        start.y = self.py
        start = PathPlanner.world_to_grid(mapdata, start)
        #start = (self.px, self.py)
        print("start", start)
        

        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        if mapdata is None:
            return Path()

        path = self.a_star(mapdata, start, goal)

        # GridCells msg for A* visited
        msg_grid_cells = GridCells()
        msg_grid_cells.header = mapdata.header

        start = PathPlanner.grid_to_world(mapdata, start[0],start[1])
        goal = PathPlanner.grid_to_world(mapdata, goal[0],goal[1])
        print("start", start)
        print("goal", goal)
        msg_grid_cells.cells.append(start)
        msg_grid_cells.cells.append(goal)
        #width and height
        res = mapdata.info.resolution
        msg_grid_cells.cell_width = res
        msg_grid_cells.cell_height = res
        print("msg",msg_grid_cells)
        self.goal.publish(msg_grid_cells)

       
        # Optimize waypoints
        print("path after Astar", path)
        waypoints = PathPlanner.optimize_path(path)
        # Return a Path message
        vals=[]
        # for i in waypoints:
        #     index = PathPlanner.grid_to_index(mapdata, waypoints[i][0], waypoints[i][1])
        #     vals.append(mapdata.data[index])

        return self.path_to_message(mapdata, waypoints)

        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        # Request the map
        # In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        # Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        # Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path = self.a_star(cspacedata, start, goal)
        # Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        # Return a Path message
        return self.path_to_message(mapdata, waypoints)

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
    PathPlanner().run()


# <node pkg="rbe3002_lab3" type="path_planner.py" name="work_lab2" />
# launch file line
