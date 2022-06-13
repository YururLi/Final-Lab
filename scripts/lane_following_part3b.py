#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
import cv2.aruco as aruco
from playsound import playsound

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
param = aruco.DetectorParameters_create()
matrix = np.array([[260.17262, 0., 160.76833],
                   [0., 260.26526, 119.20188],
                   [0., 0., 1.]])
dist = np.array([[0.163280, -0.292913, 0.002210, 0.000031, 0.000000]])


class Follower:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image',
                                          Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                           Twist, queue_size=10)
        # self.starttimer = rospy.get_time()
        self.twist = Twist()
        self.image = None
        self.gray = None
        self.previous_x = 0.0
        self.previous_z = 0.0
        self.start = 0
        self.init = 0
        self.right = 0
        self.readyright = 0
        self.left = 0
        self.straight = 0
        self.finish = 0
        self.count = 0
        self.turn0 = 0
        self.turn1 = 0
        self.turn2 = 0
        self.turn3 = 0
        self.begin = 1
        self.markerID = None
        self.aruco_found = [False] * 5
        self.audio_filename = ["0.mp3", "1.mp3", "2.mp3", "3.mp3", "4.mp3", "start_point.mp3"]
        self.s0 = rospy.get_time()
        self.s1 = rospy.get_time()
        self.s2 = rospy.get_time()
        self.s3 = rospy.get_time()
        # self.template = cv2.imread('/home/zeon/Desktop/BEV_1.png', 0).astype(np.uint8)
        self.PID_a = pid_controller(0.003, 0.00, 0)
        self.PID_b = pid_controller(3, 0.00, 0.00)
        self.stopped = False

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        corners, self.markerID, rejected = aruco.detectMarkers(self.gray, aruco_dict, parameters=param)
        matrix = np.array([[260.17262, 0., 160.76833],
                           [0., 260.26526, 119.20188],
                           [0., 0., 1.]])
        dist = np.array([[0.163280, -0.292913, 0.002210, 0.000031, 0.000000]])

        if len(corners) > 0:
            aruco_ID = self.markerID.squeeze()
            if not self.aruco_found[aruco_ID]:
                self.aruco_found[aruco_ID] = True
                playsound("/home/weixy/catkin_ws/src/lane_turtlebot3/audios/" + self.audio_filename[aruco_ID],
                          block=False)
                print(self.markerID)

    # cv2.imshow("raw",self.image)
    # cv2.waitKey(1)

    def order(self, x, z):
        self.twist.linear.x = x
        self.twist.angular.z = z
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        if self.image is None:
            rospy.loginfo("Waiting image")
            rospy.sleep(1)
            return
        top_x = 90  # 68  # 102 73     62 10 140 120     50 20 140 120    68 5 140 120
        top_y = 5  # 10 26      40 30 140 120
        bottom_x = 170  # 140  # 230
        bottom_y = 120
        # selecting 4 points from the original image
        pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y],
                            [160 - bottom_x, 120 + bottom_y]])
        IMG_H = 240
        IMG_W = 320
        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 75])
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        image_t = cv2.inRange(hsv, lower_black, upper_black)

        # homography process
        BEV = cv2.warpPerspective(image_t, h, (1000, 600))  # 10:6

        mask1 = np.copy(BEV)
        mask2 = np.copy(BEV)
        mask3 = np.copy(BEV)
        mask4 = np.copy(BEV)
        h = 600
        w = 1000
        search_top = 3 * h / 10

        mask1[:, 500:w] = 0
        mask2[:, 0:500] = 0
        mask3[180:h, :] = 0
        mask3[:, 500:w] = 0
        mask4[400:h, :] = 0
        mask4[:, 0:500] = 0
        M0 = cv2.moments(BEV)
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)
        M3 = cv2.moments(mask3)
        M4 = cv2.moments(mask4)
        # cv2.imshow("BEV", BEV)
        # cv2.imshow("mask3", mask3)
        # cv2.imshow("mask4", mask4)
        # cv2.waitKey(1)
        if self.begin == 1:
            # print(self.begin)
            if self.start == 0 and M3['m00'] == 0 and self.turn0 == 0:
                print('turn right 1')
                rospy.sleep(2)
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(0.7)
                self.twist.linear.x = 0.0
                self.twist.angular.z = -1.5
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1.1)  # 1.1
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1.5)
                self.twist.linear.x = 0.2
                self.twist.angular.z = 1.225
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(2.38)
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.start = 1
                self.turn0 = 1

            t2 = rospy.get_time()
            # print('0', t2 - self.s0)
            # print('1', t2 - self.s1)
            # print('2', t2 - self.s2)
            if t2 - self.s0 > 15:
                self.turn0 = 0
            # print("turn0=0")
            if M3['m00'] == 0 and self.finish == 0 and self.turn0 == 0:
                if self.turn1 == 0:
                    self.turn1 = 1
                    self.turn2 = 1
                    self.left = 1
                    self.s1 = rospy.get_time()
                    print('head to point 2')
                    self.begin = 0


                elif self.turn2 == 1 and t2 - self.s1 > 4:
                    self.turn2 = 0
                    self.turn3 = 1
                    self.left = 1
                    self.s2 = rospy.get_time()
                    print('left2')
                    # rospy.sleep(0.5)
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(1.675)
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 1.3
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(2.4)
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                elif self.turn3 == 1 and t2 - self.s2 > 5:
                    self.turn3 = 0
                    self.left = 1
                    self.readyright = 1  ###########
                    self.s3 = rospy.get_time()
                    print('left3')
                    # rospy.sleep(0.5)		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(1.39)
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 1.1
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(2.65)  # 2.55
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                # rospy.sleep(1)
            # t = rospy.get_time()
            # print t - self.s0

            if (M1['m00'] > 0 or M2['m00'] > 0):
                # print('normal')
                # self.left = 0
                if M1['m00'] == 0:
                    cx1 = 250
                    cy1 = 300
                    cx2 = int(M2['m10'] / M2['m00'])
                    cy2 = int(M2['m01'] / M2['m00'])
                    fpt_x = (cx1 + cx2) / 2
                    fpt_y = (cy1 + cy2) / 2
                elif M2['m00'] == 0:
                    cx1 = int(M1['m10'] / M1['m00'])
                    cy1 = int(M1['m01'] / M1['m00'])
                    cx2 = 750
                    cy2 = 300
                    fpt_x = (cx1 + cx2) / 2
                    fpt_y = (cy1 + cy2) / 2
                else:
                    cx1 = int(M1['m10'] / M1['m00'])
                    cy1 = int(M1['m01'] / M1['m00'])
                    cx2 = int(M2['m10'] / M2['m00'])
                    cy2 = int(M2['m01'] / M2['m00'])
                    fpt_x = (cx1 + cx2) / 2
                    fpt_y = (cy1 + cy2) / 2
                self.twist.linear.x = 0.2

                cv2.circle(BEV, (cx1, cy1), 10, (100, 255, 255), -1)
                cv2.circle(BEV, (cx2, cy2), 10, (100, 255, 255), -1)
                cv2.circle(BEV, (fpt_x, fpt_y), 10, (255, 100, 100), -1)

                err = 10 + w / 2 - fpt_x

                alpha = -np.arctan2(fpt_x - w / 2, h - fpt_y)

                self.twist.angular.z = alpha * 1.5
                # self.twist.angular.z = err*0.008

                self.cmd_vel_pub.publish(self.twist)

            # else:
            # self.twist.linear.x = self.previous_x
            # self.twist.angular.z = self.previous_z

            thistime = rospy.get_time()
            # print(thistime - self.s1)
            if self.readyright == 1 and self.right == 0 and M4['m00'] == 0 and self.finish == 0:
                self.right = 1
            if self.right == 1 and M4['m00'] > 0 and abs(self.s2 - thistime) > 4:
                print("turn right 2")
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1.785)
                self.twist.linear.x = 0.0
                self.twist.angular.z = -1.5
                self.cmd_vel_pub.publish(self.twist)
                self.right = 0
                self.finish = 1
                rospy.sleep(1.1)
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
            if self.finish == 1 and M0['m00'] == 0:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1.5)
                # self.twist.linear.x = 0.0
                # self.twist.angular.z = 0.0
                # self.cmd_vel_pub.publish(self.twist)
                self.begin = 0
        # cv2.imshow("homo", BEV)
        # cv2.waitKey(1)


class navigator:
    def __init__(self):
        self.goalPoints = [
            # from point1 to point2, to point3, to point4 and then back to point1
            # position[XYZ] and pose[quaternion]
            # In our map of lab, increase x goes up, increase y goes left

            # [(4.15, -0.05, 0.0), (0.0, 0.0, 0.737849902397, 0.674964829848)], # 2
            # [(4.4, 3.8, 0.0), (0.0, 0.0, 0.025369587372, 0.999678140221)], # 3
            # [(0.26703, 4.1, 0.0), (0.0, 0.0, 0.99998940623, 0.00460298028122)], # 4
            # [(0.025, 0, 0.0), (0.0, 0.0, 0.73358, 0.67959)], # 0
            # [(0.95, 0.76625, 0.0), (0.0, 0.0, 0.0456504303085, 0.9999999)], # lane_follow
            # [(1.66254192403, 0.647986888368, 0.0), (0.0, 0.0, 0.999174038183, 0.0406354699776)] # cross
            # [(0.063593920772, 0.0771376749818, 0.00), (0.000, 0.000, 0.000, 1.000)], #point 2  straight
            [(0.139999901056, 0.12999984622, 0), (0, 0, -0.698, 0.716)],
            [(-0.2049999547, -3.91999936104, 0.00), (0.000, 0.000, 0.000, 1.000)],  # point 3
            [(3.87499880791, -4.42999982834, 0.00), (0.000, 0.000, 0.99897277669, 0.0453143623188)],  # point 4
            [(4.19958301544, -0.264779472351, 0.00), (0.000, 0.000, -0.708046509431, 0.706165802402)],
            # point 1 #map_new
            [(2.20499968529, -0.860000371933, 0.0), (0.0, 0.0, 0.67721942459, 0.735781116201)],  # begin
            # [(3.22499966621, -0.980000257492, 0.0), (0.0, 0.0, 0.999043220277, 0.0437337857891)], # lane_follow near
            # [(3.4049996376, -0.970000326633, 0.0), (0.0, 0.0, 0.999646184863, 0.0437337857891)], # lane_follow far
            [(3.4049996376, -1.020000326633, 0.0), (0.0, 0.0, 0.999646184863, 0.0437337857891)],
            # lane_follow far2 fine tune

            [(0.224999770522, -0.320000221729, 0.0), (0.0, 0.0, -0.710779897245, 0.703414484975)]  # point2 start

        ]
        self.status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.callback)
        self.status_list = []
        self.status = -1

    # rospy.Subscriber('move_base/status', GoalStatusArray, self.callback)
    # rospy.Publisher()

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id(), data)
        # flag = data.status_list[-2]
        self.status_list = data.status_list
        if self.status_list:
            self.status = self.status_list[0].status

    def init_send(self, pose):
        init_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, latch=True, queue_size=1)
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = 'map'
        init_msg.pose.pose.position.x = pose[0][0]
        init_msg.pose.pose.position.y = pose[0][1]
        init_msg.pose.pose.position.z = pose[0][2]
        init_msg.pose.pose.orientation.x = pose[1][0]
        init_msg.pose.pose.orientation.y = pose[1][1]
        init_msg.pose.pose.orientation.z = pose[1][2]
        init_msg.pose.pose.orientation.w = pose[1][3]
        init_pub.publish(init_msg)

    def goal_send(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = pose[0][0]
        goal.target_pose.pose.position.y = pose[0][1]
        goal.target_pose.pose.position.z = pose[0][2]
        goal.target_pose.pose.orientation.x = pose[1][0]
        goal.target_pose.pose.orientation.y = pose[1][1]
        goal.target_pose.pose.orientation.z = pose[1][2]
        goal.target_pose.pose.orientation.w = pose[1][3]
        return goal

    def navigation(self, num):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server(rospy.Duration(3))

        goal = self.goal_send(self.goalPoints[num])
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(120))
        print(num)


class pid_controller:

    def __init__(self, p_coef, i_coef, d_coef):
        self.kp = p_coef
        self.ki = i_coef
        self.kd = d_coef
        self._previous_error = 0.0
        self.sum = 0.0

    def set_current_error(self, error):
        output0 = error * self.kp

        error_diff = error - self._previous_error
        output1 = self.kd * error_diff

        self.sum += error
        output2 = self.ki * self.sum

        self._previous_error = error

        output = output0 + output1 + output2

        return output


if __name__ == "__main__":
    rospy.init_node('navigation')

    n = navigator()
    c_rate = rospy.Rate(30)
    # n.init_send(n.goalPoints[3])

    # for i in range(4):
    # 	follower = Follower()

    # 	rospy.sleep(1)
    # 	n.navigation(5)
    # 	rospy.sleep(1)
    # 	n.navigation(3)
    # 	rospy.sleep(1)
    # 	# n.navigation(2)
    # 	# rospy.sleep(1)
    # 	# n.navigation(3)

    # rospy.sleep(1)
    # n.navigation(0)

    follower = Follower()
    rospy.sleep(1)
    follower.s0 = rospy.get_time()
    while follower.begin == 1:
        follower.run()
    # c_rate.sleep()
    follower.order(0, 0)
    rospy.sleep(0.5)

# for i in range(4):

# 	follower = Follower()
# 	# initialized
# 	rospy.sleep(1)
# 	# follower.run()
# 	# follower.order(0,1.5)
# 	# rospy.sleep(4.23)
# 	# follower.order(0,0)
# 	# rospy.sleep(1)

# 	n.navigation(5)
# 	playsound("/home/weixy/catkin_ws/src/lane_turtlebot3/audios/start_point.mp3",block=False)
# 	rospy.sleep(2)

# 	# follower.run()

# 	rospy.sleep(1)
# 	follower.s0 = rospy.get_time()
# 	while follower.begin == 1:
# 		follower.run()
# 		# c_rate.sleep()
# 	follower.order(0,0)
# 	rospy.sleep(0.5)

# 	follower.order(0.19,-0.135)
# 	rospy.sleep(3.3)
# 	follower.order(0,0)
# 	rospy.sleep(1)

# 	follower.order(0,1.5)
# 	rospy.sleep(1.6)
# 	follower.order(0,0)
# 	rospy.sleep(1)

# 	follower.order(0.19,-0.135)
# 	rospy.sleep(3.3)
# 	follower.order(0,0)
# 	rospy.sleep(1.5)

# 	# n.navigation(0)
# 	# rospy.sleep(1)
# 	# n.navigation(0)
# 	follower.s1 = rospy.get_time()
# 	follower.begin = 1
# 	while follower.begin == 1:
# 		follower.run()
# 		# c_rate.sleep()
# 	follower.order(0,0)

# 	n.navigation(3)
# 	playsound("/home/weixy/catkin_ws/src/lane_turtlebot3/audios/finish.mp3",block=False)

# if follower.begin == 0:
# 	rospy.sleep(1)
# 	n.navigation(1)
# 	rospy.sleep(1)
# 	n.navigation(2)
# 	# rospy.sleep(1)
# 	# n.navigation(0)
# 	# rospy.sleep(1)
# 	# n.navigation(5)
# 	rospy.sleep(1)
# 	n.navigation(3)
# rospy.sleep(2)
# print('start')
