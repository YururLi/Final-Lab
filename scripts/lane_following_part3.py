#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from sensor_msgs.msg import Image
import cv2.aruco as aruco



aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
param = aruco.DetectorParameters_create()
matrix = np.array([[260.17262,   0.     , 160.76833],
					[0.     , 260.26526, 119.20188],
					[0., 0., 1.]])
dist = np.array([[0.163280, -0.292913, 0.002210, 0.000031, 0.000000]])
class Follower:
	def __init__(self):

		self.bridge = cv_bridge.CvBridge()

		self.image_sub = rospy.Subscriber('camera/image',
										Image, self.image_callback)

		self.cmd_vel_pub = rospy.Publisher('cmd_vel',
										Twist, queue_size=10)
		self.starttimer = rospy.get_time()
		self.twist = Twist()
		self.image = None
		self.gray = None
		self.previous_x = 0.0
		self.previous_z = 0.0
		self.start = 0
		self.init = 0
		self.right = 0
		self.left = 0
		self.straight = 0
		self.finish = 0
		self.count = 0
		self.turn0 = 0
		self.turn1 = 0
		self.turn2 = 0
		self.begin = 1
		self.s0 = rospy.get_time()
		self.s1 = rospy.get_time()
		# self.template = cv2.imread('/home/zeon/Desktop/BEV_1.png', 0).astype(np.uint8)
		self.PID_a = pid_controller(0.003, 0.00, 0)
		self.PID_b = pid_controller(2, 0.00, 0.00)
		self.stopped = False
	def image_callback(self, msg):
		self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
		corners, markerID, rejected = aruco.detectMarkers(self.gray, aruco_dict, parameters=param)
		matrix = np.array([[260.17262,   0.     , 160.76833],
						[0.     , 260.26526, 119.20188],
						[0., 0., 1.]])
		dist = np.array([[0.163280, -0.292913, 0.002210, 0.000031, 0.000000]])

		if len(corners) > 0:
			rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, matrix, dist)
			(rvec - tvec).any()
			for i in range(rvec.shape[0]):
				aruco.drawDetectedMarkers(self.image, corners, markerID)
				print("[INFO] ArUco marker ID: {}".format(markerID))


	def run(self):
		if self.image is None:
			rospy.loginfo("Waiting image")
			rospy.sleep(1)
			return
		top_x = 68  # 102 73     62 10 140 120     50 20 140 120    68 5 140 120
		top_y = 5  # 10 26      40 30 140 120
		bottom_x = 140  # 230
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
		upper_black = np.array([180, 255, 80])
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
		mask3[200:h, :] = 0
		mask3[:, 500:w] = 0
		mask4[400:h, :] = 0
		mask4[:, 0:500] = 0
		M0 = cv2.moments(BEV)
		M1 = cv2.moments(mask1)
		M2 = cv2.moments(mask2)
		M3 = cv2.moments(mask3)
		M4 = cv2.moments(mask4)
		if self.begin == 1:
			if self.start == 0 and M3['m00'] == 0 and self.turn0 == 0:
				rospy.sleep(2)
				self.twist.linear.x = 0.22
				self.twist.angular.z = 0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(0.6)
				self.twist.linear.x = 0.0
				self.twist.angular.z = -1.57
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				self.twist.linear.x = 0.22
				self.twist.angular.z = 0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1.4)
				self.twist.linear.x = 0.22
				self.twist.angular.z = 1.25
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(2.8)
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				self.start = 1
				self.turn0 = 1

			t2 = rospy.get_time()
			print(t2 - self.starttimer)
			if t2 - self.starttimer > 25:
				self.turn0 = 0
			if M3['m00'] == 0 and self.left == 0 and self.finish == 0 and self.turn0 == 0:
				if self.turn1 == 0:
					self.turn1 = 1
					self.turn2 = 1
					self.left = 1
					self.s0 = rospy.get_time()
					print('left1')
					#rospy.sleep(0.5)
					self.twist.linear.x = 0.22
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(1.33)
					self.twist.linear.x = 0.2
					self.twist.angular.z = 1.1
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(2.9)
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)
				elif self.turn2 == 1 and t2 - self.s0 > 7:
					self.turn2 = 0
					self.left = 1
					self.s0 = rospy.get_time()
					print('left2')
					#rospy.sleep(0.5)
					self.twist.linear.x = 0.22
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(1.7)
					self.twist.linear.x = 0.2
					self.twist.angular.z = 1.3
					self.cmd_vel_pub.publish(self.twist)
					rospy.sleep(2.4)
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					self.cmd_vel_pub.publish(self.twist)

			t = rospy.get_time()
		#print t - self.s0

			if (M1['m00'] > 0 or M2['m00'] > 0) and t - self.s0 > 4.1:
				self.left = 0
				if M1['m00'] == 0:
					cx1 = 150
					cy1 = 300
					cx2 = int(M2['m10'] / M2['m00'])
					cy2 = int(M2['m01'] / M2['m00'])
					fpt_x = (cx1 + cx2) / 2
					fpt_y = (cy1 + cy2) / 2
				elif M2['m00'] == 0:
					cx1 = int(M1['m10'] / M1['m00'])
					cy1 = int(M1['m01'] / M1['m00'])
					cx2 = 850
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
				self.twist.linear.x = 0.22

				cv2.circle(BEV, (cx1, cy1), 10, (100, 255, 255), -1)
				cv2.circle(BEV, (cx2, cy2), 10, (100, 255, 255), -1)
				cv2.circle(BEV, (fpt_x, fpt_y), 10, (255, 100, 100), -1)

				err = 10 + w / 2 - fpt_x

				alpha = -np.arctan2(fpt_x - w / 2, h - fpt_y)
			
				self.twist.angular.z = alpha

				self.cmd_vel_pub.publish(self.twist)

			# else:
			# self.twist.linear.x = self.previous_x
			# self.twist.angular.z = self.previous_z

			thistime = rospy.get_time()
			# print(thistime - self.starttimer)
			if thistime - self.starttimer >= 40 and self.right == 0 and M4['m00'] == 0 and self.finish == 0:
				self.right = 1
			if self.right == 1 and M4['m00'] > 0:
				print("detecting right")
				self.twist.linear.x = 0.2
				self.twist.angular.z = 0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(2)
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
				self.twist.linear.x = 0.22
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1.5)
				# self.twist.linear.x = 0.0
				# self.twist.angular.z = 0.0
				# self.cmd_vel_pub.publish(self.twist)
				self.begin = 0
			cv2.imshow("BEV", BEV)
			cv2.waitKey(1)

class navigator:
	def __init__(self):
		self.goalPoints = [ 
			# from point1 to point2, to point3, to point4 and then back to point1
			# position[XYZ] and pose[quaternion]
			# In our map of lab, X-direction is from bottom to top and Y-direction is from right to left
			
			[(4.15, -0.120, 0.0), (0.0, 0.0, -0.725395177838, 0.688332649211)], # 2
			[(4.4, 3.8, 0.0), (0.0, 0.0, 0.025369587372, 0.999678140221)], # 3
			[(0.26703, 4.1, 0.0), (0.0, 0.0, 0.99998940623, 0.00460298028122)], # 4
			[(0.03, 0.000, 0.0), (0.0, 0.0, 0.73358, 0.67959)], # 0
			[(0.755, 0.77670139439, 0.0), (0.0, 0.0, 0, 0.999947272788)], # lane_follow
			[(1.66254192403, 0.647986888368, 0.0), (0.0, 0.0, 0.999174038183, 0.0406354699776)] # cross
			
		]	
		self.status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.callback)
		self.status_list = []
		self.status = -1
		#rospy.Subscriber('move_base/status', GoalStatusArray, self.callback)
	# rospy.Publisher()

	def callback(self,data):
		# rospy.loginfo(rospy.get_caller_id(), data)
		# flag = data.status_list[-2]
		self.status_list = data.status_list
		if self.status_list:
			self.status = self.status_list[0].status

	def init_send(self,pose):
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


	def goal_send(self,pose):
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

	def navigation(self,num):
		client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
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

if __name__=="__main__":
	rospy.init_node('navigation')
	
	n = navigator()
	follower = Follower()
	# cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	# init_send(goalPoints[4])
	# twist = Twist()
	# rospy.sleep(2)
	# twist.linear.x = 0.0status
	# twist.angular.z = -0.2
	# cmd_vel_pub.publish(twist)
	# rospy.sleep(33)
	# twist.linear.x = 0.0
	# twist.angular.z = 0.0
	# cmd_vel_pub.publish(twist)	
	#print(status)
    	#rospy.spin()
	#navigation(1)
	n.navigation(4)
	if n.status == 3:
		rospy.sleep(1)
		print(n.status)
		
	while follower.begin == 1:
		follower.run()
	
	if follower.begin == 0:
		rospy.sleep(1)
		n.navigation(1)
		rospy.sleep(1)
		n.navigation(2)
		# rospy.sleep(1)
		# n.navigation(0)
		# rospy.sleep(1)
		# n.navigation(5)
		rospy.sleep(1)
		n.navigation(3)
	#rospy.sleep(2)
	#print('start')
	

	
				

