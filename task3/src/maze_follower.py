#!/usr/bin/env python3 

from distutils.log import error
from turtle import left, right

import numpy as np
import rospy
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
import waffle
import math
import time
class MazeFollower(object):
    def __init__(self):
        self.rate = rospy.Rate(10) # Hz
        self.left_wall_control = 0.0
        self.right_wall_control = 0.0
        self.motion = waffle.Motion(debug=True)
        self.lidar = waffle.Lidar(debug=True)
        self.odom = waffle.Pose(debug=True)
        self.error = 0.08
        self.fwd_vel = 0.2
        self.ang_vel = math.pi/6
        self.turn_time = 1.3
        self.right_wall_desired_dist = 0.4
        self.error = 0.05
        self.intended_direction = self.odom.yaw * self.odom.yaw_direction
    
    def stop(self):
        self.motion.set_velocity(self.fwd_vel, 0.0)
        self.motion.publish_velocity()
        self.motion.stop()
        time.sleep(1)

    def correction(self):
        left_wall = min(self.lidar.subsets.l3Array)
        right_wall = min(self.lidar.subsets.r3Array)
        if right_wall < 0.4 and left_wall < 0.4:
            average_dist = (left_wall + right_wall) / 2 

            error_left = average_dist - left_wall
            error_right = average_dist - right_wall

            # since turning left is positive and right is negative
            correction = error_right - error_left
        else:
            if right_wall < 0.4 and right_wall > 0.0:
                correction = 0.25 - right_wall
            elif left_wall < 0.4 and left_wall > 0.0:
                correction = 0.25 - left_wall
            else:
                correction = 0
        
        self.motion.set_velocity(self.fwd_vel, correction/1.5)
        self.motion.publish_velocity()


    def moveForward(self):
        front_dist = min(self.lidar.subsets.frontArray)
        while front_dist > 0.4:
            print("Going forward")
            self.correction()
            front_dist = min(self.lidar.subsets.frontArray)
        self.stop()
        time.sleep(1)
        return
    
    def face_wall(self):
        sign = np.sign((self.odom.yaw * self.odom.yaw_direction) - self.intended_direction)
        while abs((self.odom.yaw * self.odom.yaw_direction) - self.intended_direction) > 0.5:
            self.motion.set_velocity(0.0, (sign * self.ang_vel))
            self.motion.publish_velocity()
        

    def turn_90(self, direction):
        # TODO Fix turning logic!!
        self.face_wall()
        current_yaw = self.odom.yaw
        rounded_yaw = round(current_yaw / 90) * 90
        print(f"Current yaw: {current_yaw}")
        print(f"Rounded yaw: {rounded_yaw}")

        if direction == "right":
            
            target_yaw = rounded_yaw - 90
            ang_vel = -self.ang_vel
        elif direction == "left":
            target_yaw = rounded_yaw + 90
            ang_vel = self.ang_vel

        print(f"Target yaw: {target_yaw}")
        

        while abs((self.odom.yaw * self.odom.yaw_direction) - target_yaw) > 0.5:
            print(self.odom.yaw, self.odom.yaw_direction)
            self.motion.set_velocity(0.0, ang_vel)
            self.motion.publish_velocity()
        
        self.stop()
        self.intended_direction = target_yaw
            

    def run(self):
        self.moveForward()
        while not rospy.is_shutdown():
            self.lidar.subsets.show()
            self.right_wall = min(self.lidar.subsets.r3Array)
            self.left_wall = min(self.lidar.subsets.l3Array)
            self.front_wall = min(self.lidar.subsets.frontArray)
            # TODO: fix correction algorithm
            # TODO: turn 90 can be improved by using odom data better somehow
            # if there is no wall on the right, turn right
            if self.right_wall > 0.4:
                self.stop()
                self.turn_90("right")
                self.moveForward()
                print("Turning right")
            # if there is wall in front and a wall on the right, turn left
            elif self.front_wall < 0.4:
                self.turn_90("left")
                print("Turning left")
                self.moveForward()
            # if there is no wall in the front, and there is a right wall, keep going forward
            else:
                self.moveForward()
                
        
if __name__ == '__main__':
    rospy.init_node('maze_follower')
    maze_follower = MazeFollower()
    maze_follower.run()
    rospy.spin()
    rospy.on_shutdown((maze_follower.motion.stop(), maze_follower.motion.publish_velocity()))
        
        
        