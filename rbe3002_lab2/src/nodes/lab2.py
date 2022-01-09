#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from nav_msgs.srv import GetPlan


class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=6)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # initialize variables
        self.pth = 0
        self.px = 0
        self.py = 0
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.done = rospy.Publisher('/done_driving', Bool, queue_size=5)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        ## incoming path message
        rospy.Subscriber('/go_to_path', Path, self.go_to_path)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.handle_initial_pose)
        self.initial_pose = PoseStamped()
        rospy.sleep(1)
        print('node ready')

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        msg_cmd_vel = Twist()
        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        ### Publish the message
        self.cmd_vel.publish(msg_cmd_vel)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        Xi = self.px
        Yi = self.py
        thetai = self.pth
        self.send_speed(linear_speed, 0)
        done = False
        while not done:
            if abs(abs(distance) - math.sqrt((self.px - Xi) ** 2 + (self.py - Yi) ** 2)) < 0.05:
                self.send_speed(0, 0)
                done = True
            else:
                rospy.sleep(0.5)

    def normalize_angles(self, a):
        while a < -math.pi:
            a = a + 2*math.pi
        while a > math.pi:
            a = a - 2*math.pi
        # print(a)
        return a

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ROTATION_TOLERANCE = math.pi * 4.0 / 180.0
        angle = self.normalize_angles(angle)
        print("angle")
        print(angle)
        if angle < 0:
            aspeed = -aspeed
            print("aspeed")
            print(aspeed)
        print("aspeed")
        print(aspeed)
        theta0 = self.pth
        print("initial theta")
        print(theta0)
        self.send_speed(0, aspeed)
        while abs(self.normalize_angles(self.normalize_angles(self.pth - theta0) - angle)) > ROTATION_TOLERANCE:
            print("new theta")
            print(self.pth)
            rospy.sleep(0.5)
        print("done rotating")
        self.send_speed(0, 0)

    def handle_initial_pose(self, msg):
        pose = Pose()
        pose.position = msg.pose.pose.position
        start = PoseStamped()
        start.header = msg.header
        start.pose = pose
        self.initial_pose = start

    def initial_pose_robot(self, msg):
        pose = Pose()
        point = Point()
        point.x = self.px
        point.y = self.py
        point.z = 0.0
        pose.position = point
        start = PoseStamped()
        start.header = msg.header
        start.pose = pose
        self.initial_pose = start

    def request_path(self, msg, version):
        if version == 1:
            self.initial_pose_robot(msg)
        path = []
        rospy.wait_for_service('/path_plan')
        try:
            get_plan = rospy.ServiceProxy('/path_plan', GetPlan)
            path = get_plan(self.initial_pose, msg, 0).plan.poses
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
        rospy.loginfo("requesting path")
        return path

    def go_to_path(self, msg):
        # path = self.request_path(msg, 1)
        self.initial_pose_robot(msg)
        path = msg.poses

        print("go_to_path")
        for i in range(len(path)):
            # print('round', i)
            print("pose: ", path[i])
            self.go_to(path[i])
            rospy.sleep(1)
        self.done.publish(True)

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # initial position
        x = self.px
        y = self.py
        theta = self.pth
        print("curr x,y,theta", x, y, theta)
        # message x and y
        targx = msg.pose.position.x
        targy = msg.pose.position.y
        # distance
        d = math.sqrt(((targx - x) ** 2 + (targy - y) ** 2))

        # msg angle
        quat_orig = msg.pose.orientation
        # print("quat", quat_orig)
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        targtheta = yaw

        print("targ x,y,theta", targx, targy, targtheta)
        ym = targy - self.py
        xm = targx - self.px
        print("y", ym)
        print("x", xm)
        # find the turning angle to line up with the second point
        angle1 = math.atan2(ym, xm)
        ang = angle1 - theta
        print("angle given in go_to", ang)
        # theta1 = angle1 - theta
        self.drive(-0.05, 0.05)
        self.rotate(ang, 0.25)
        # print("done with turn 1")

        # drive the distance
        self.drive(d, 0.08)
        # print("done driving")

        # rotate to final implementation
        # curTheta = self.pth
        # angle2 = targtheta - curTheta
        # self.rotate(targtheta, 0.25)
        # print("done with turn 2")

        # print("curr x,y,theta after go_to", x,y,theta)
        # print(targx, targy, targtheta)

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

    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass  # delete this when you implement your code

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass  # delete this when you implement your code

    def run(self):
        print("rotating")
        # self.normalize_angles(3*math.pi/2)
        # self.rotate(math.pi / 4, 0.25)
        rospy.spin()


if __name__ == '__main__':
    Lab2().run()
