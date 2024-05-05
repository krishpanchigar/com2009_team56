#!/usr/bin/env python3

import math
import rospy
import tf
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError
# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
# Import some other modules from within this package
from tb3 import Tb3Move

class RobotSLAM:
    def __init__(self):
        rospy.init_node('exploration', anonymous=False)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)    

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

        self.target_colour = rospy.get_param('~target_colour', 'green')

        # self.color_ranges = {
        #     'green': ((81, 175, 0), (92, 255, 255)),
        #     'blue': ((115, 224, 100), (130, 255, 255)),
        #     'red': ((0, 175, 0), (7, 255, 255)),
        #     'yellow': ((23, 175, 0), (28, 250, 255)),
        # }
        rospy.loginfo("Target Colour: %s", self.target_colour)

    def set_initial_pose(self, x, y, theta):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))
        
        self.initial_pose_pub.publish(initial_pose)
        rospy.sleep(1)
    
    def odometry_callback(self, msg):
        orientation_quaternion = msg.pose.pose.orientation
        orientation_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.robot_position = msg.pose.pose.position

        self.robot_yaw = yaw

    def move_to_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 300
        crop_x = int((width / 2) - (crop_width / 2))
        crop_y = int((height / 2) - (crop_height / 2))
        crop_img = cv_img[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.target_colour == "green":
            lower = (81, 175, 0)
            upper = (92, 255, 255)
        elif self.target_colour == "red":
            lower = (0, 175, 0)
            upper = (7, 255, 255)
        elif self.target_colour == "yellow":
            lower = (23, 175, 0)
            upper = (28, 250, 255)
        elif self.target_colour == "blue":
            lower = (115, 224, 100)
            upper = (130, 255, 255)

        mask = cv2.inRange(hsv_img, lower, upper)
        m = cv2.moments(mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(crop_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    focal_length = 1206.8897719532354
                    real_object_width = 200
                    distance_mm = (focal_length * real_object_width) / w
                    distance_m = distance_mm / 1000
                    rospy.loginfo(f"Estimated distance: {distance_m} m")

                    object_x = distance_m * math.cos(self.robot_yaw)
                    object_y = distance_m * math.sin(self.robot_yaw)
                    self.object_x_global = self.robot_position.x + object_x
                    self.object_y_global = self.robot_position.y + object_y

                    rospy.loginfo("Calculated global position: x=%f, y=%f", self.object_x_global, self.object_y_global)

                    self.waypoint_ready = True
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            # if self.stop_counter > 0:
            #     self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
    

if __name__ == '__main__':
    robot_slam = RobotSLAM()
    
    try:
        robot_slam.main()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exploration and map saving complete.")
    rospy.signal_shutdown("Exploration and map saving complete.")
