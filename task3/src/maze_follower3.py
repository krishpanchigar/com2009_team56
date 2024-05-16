#!/usr/bin/env python3

import rospy
from waffle import Motion, Pose, Lidar 

class Maze_follower:
    def __init__(self):
        self.node_name = "follow_wall"
        rospy.init_node(self.node_name, anonymous=True)

        self.motion_controller = Motion(debug=True)
        self.pose_controller = Pose(debug=True)
        self.lidar_controller = Lidar(debug=True)
        
        self.rate = rospy.Rate(10) 


    def maze_follow(self):
        while not rospy.is_shutdown():
            front_distance = min(self.lidar_controller.subsets.frontArray)
            right_distance = min(self.lidar_controller.subsets.r2Array) 

            if front_distance > 0.4:
                if right_distance < 0.3:
                    rospy.loginfo("Turning left")
                    self.motion_controller.set_velocity(linear=0.26, angular=0.9)
                elif 0.3 <= right_distance < 0.4:
                    rospy.loginfo("Going straight")
                    self.motion_controller.set_velocity(linear=0.26, angular=0)
                else:
                    rospy.loginfo("Turning right")
                    self.motion_controller.set_velocity(linear=0.26, angular=-0.9)
            else:
                rospy.loginfo("Front obstacle detected. Turning away.")
                self.motion_controller.set_velocity(linear=0, angular=1.2)
                self.motion_controller.publish_velocity()

            self.motion_controller.publish_velocity() 
            self.rate.sleep()

if __name__ == '__main__':
    maze_solver = Maze_follower()
    maze_solver.solve_maze()
