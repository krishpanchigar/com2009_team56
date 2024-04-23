#!/usr/bin/env python3

import rospy
import tf
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
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
        rospy.init_node('robot_slam', anonymous=False)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

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

        self.color_detections = {}

        self.color_ranges = {
            'green': ((81, 175, 0), (92, 255, 255)),
            'blue': ((115, 224, 100), (130, 255, 255)),
            'red': ((0, 175, 0), (7, 255, 255)),
            'yellow': ((23, 175, 0), (28, 250, 255)),
        }

    def set_initial_pose(self, x, y, theta):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))
        
        self.initial_pose_pub.publish(initial_pose)
        rospy.sleep(1)

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

    def save_map(self, map_name):
        map_saver_command = "rosrun map_server map_saver -f " + map_name
        os.system(map_saver_command)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        for color, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv_img, lower, upper)
            result_img = cv2.bitwise_and(crop_img, crop_img, mask=mask)
            
            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)

            if self.m00 > self.m00_min: 
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                print(f"Detected {color} at y-position {self.cy}")
                self.color_detections[color] = (self.m00, self.cy)

            cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)
    

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            detected = False

            for color, (m00, cy) in self.color_detections.items():
                if m00 > self.m00_min:
                    if 460 <= cy <= 660:
                        self.move_rate = 'stop'
                        self.stop_counter = 30 
                        print(f"STOPPED: The blob of {color} is now dead-ahead at y-position {cy} pixels... Counting down: {self.stop_counter}")
                        detected = True
                        break
                    else:
                        print(f"MOVING SLOW: A blob of {color} of size {m00:.0f} pixels is in view at y-position: {cy:.0f} pixels.")
                        self.move_rate = 'slow'
                        detected = True

            if not detected: 
                if self.move_rate != 'fast':
                    print("MOVING FAST: No relevant objects in view, scanning the area...")
                    self.move_rate = 'fast'

            # Apply the current movement command based on move_rate
            if self.move_rate == 'stop' and self.stop_counter > 0:
                self.robot_controller.set_move_cmd(0.0, 0.0)
            elif self.move_rate == 'slow':
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'fast':
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

            self.robot_controller.publish()
            self.rate.sleep()

# Main execution
if __name__ == '__main__':
    robot_slam = RobotSLAM()

    # robot_slam.set_initial_pose(0, 0, 0)

    # waypoints = [(-1.6,1.6)] 

    # for point in waypoints:
    #     result = robot_slam.move_to_goal(point[0], point[1])
    #     if result:
    #         rospy.loginfo("Reached waypoint %s", point)
    #     else:
    #         rospy.loginfo("Failed to reach waypoint %s", point)
    
    
    try:
        robot_slam.main()
    except rospy.ROSInterruptException:
        pass

    # Save the map at the end of exploration
    # robot_slam.save_map("my_robot_map")

    rospy.loginfo("Exploration and map saving complete.")
    rospy.signal_shutdown("Exploration and map saving complete.")
