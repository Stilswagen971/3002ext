#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from std_msgs.msg import Bool
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from rbe3002_lab3.srv import path_selection
from rbe3002_lab4.srv import get_centroids, get_cspace

class Exec_Node:
    def __init__(self):
        rospy.init_node('exec_node')
        # subscribe to cspace topic

        self.go_to = rospy.Publisher('/go_to_path', Path, queue_size =5)
        self.MAP = rospy.Publisher('test_map', OccupancyGrid, queue_size =5)
        rospy.Subscriber('/done_driving', Bool, self.move)
        self.done= False

        print("exec node ready")


    def move(self,msg):
      self.done = True


    @staticmethod
    def request_centroid(cspace):
            centroid_list = []
            rospy.wait_for_service('/centroid_list')
            try:
                
                Get_centroids = rospy.ServiceProxy('/centroid_list', get_centroids)
                centroid_list = Get_centroids(cspace).centroids
                
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
                #print(cspace)
            rospy.loginfo("requesting centriods")
            return centroid_list
            
    @staticmethod
    def request_frontier_path( centroid_list, cspace):
            frontier_path = Path()
            rospy.wait_for_service('/frontier_path')
            try:
                Path_selection = rospy.ServiceProxy('/frontier_path', path_selection)
                frontier_path = Path_selection(centroid_list, cspace).path
                
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
            rospy.loginfo("requesting frontier path")
            return frontier_path
            
    @ staticmethod
    def request_cspace (mapdata):
            frontier_path = Path()
            rospy.wait_for_service('/calc_cspace')
            try:
                Get_cspace = rospy.ServiceProxy('/calc_cspace', get_cspace)
                cspace = Get_cspace(mapdata).cspace
                
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
            rospy.loginfo("requesting cspace")
            return cspace
            
    # def checkCentroids(self, centroid_list):
    #     if centroid_list[0].x == 0 and centroid_list[0].y == 0 and centroid_list[0].y == 0:
    #         return False
    #     else:
    #         return True
    @ staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.wait_for_service('/dynamic_map')
        try:
            map_proxy = rospy.ServiceProxy('/dynamic_map', GetMap)
            resp = map_proxy()
            return resp.map
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
        rospy.loginfo("Requesting the map")


    def loop(self):
        # rospy.loginfo("in the loop")
        print("first")
        #request the map
        mapdata = Exec_Node.request_map()
        print("got map")
        #request the cspace
        cspace = Exec_Node.request_cspace(mapdata)
        print("got cspace")
        #request the frontier centroids
        centroids = Exec_Node.request_centroid(cspace)
        print("centroids", centroids)
        # path_to_frontier = Exec_Node.request_frontier_path(centroids,cspace)
        # print("path",path_to_frontier)
        #rospy.sleep(10)
        # if self.checkCentroids(centroids):
        print('calculating path')
        path_to_frontier = Exec_Node.request_frontier_path(centroids, cspace)
        

        self.go_to.publish(path_to_frontier)

        print("should be moving")
        while self.done == False:
            rospy.sleep(1)
            #print("waiting")
        print("loop again")
        self.done = False
        

        # else:
        #     #save map
        #     print('save map')
        






    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        while not rospy.is_shutdown():
            self.loop()
        print("all done!")
        rospy.spin()




if __name__ == '__main__':
    Exec_Node().run()