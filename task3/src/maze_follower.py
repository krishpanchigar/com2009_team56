#!/usr/bin/env python3 

from distutils.log import error
from turtle import left, right
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
        self.fwd_vel = 0.15
        self.ang_vel = math.pi/6
        self.turn_time = 1.3
        self.right_wall_desired_dist = 0.4
        self.error = 0.05
    
    def stop(self):
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
            if right_wall < 0.4:
                correction = 0.25 - right_wall
            elif left_wall < 0.4:
                correction = 0.25 - left_wall
            else:
                correction = 0
        
        self.motion.set_velocity(self.fwd_vel, correction)
        self.motion.publish_velocity()


    def moveForward(self):
        front_dist = min(self.lidar.subsets.frontArray)
        while front_dist > 0.35:
            print("Going forward")
            self.correction()
            front_dist = min(self.lidar.subsets.frontArray)
        self.stop()
        time.sleep(1)
        return

    def turn_90(self, direction):
        current_yaw = self.odom.yaw
        rounded_yaw = round(current_yaw / 90) * 90
        print(f"Rounded yaw: {rounded_yaw}")

        if direction == "right":
            
            target_yaw = rounded_yaw - 90
            ang_vel = -self.ang_vel
        elif direction == "left":
            target_yaw = rounded_yaw + 90
            ang_vel = self.ang_vel

        rospy.loginfo(f"Target yaw: {target_yaw}")
        

        while abs(self.odom.yaw - target_yaw) > 1:
            self.motion.set_velocity(0.0, ang_vel)
            self.motion.publish_velocity()
        
        self.stop()

    def turn(self, angle):
        self.motion.set_velocity(0.0, angle)
        self.motion.publish_velocity()
        stop = False
        frontLast = min(self.lidar.subsets.frontArray)
        while not stop:
            frontCurrent = min(self.lidar.subsets.frontArray)
            if frontCurrent < frontLast:
                stop = True
            else:
                frontLast = frontCurrent
        self.stop()
            

    def run(self):
        self.moveForward()
        while not rospy.is_shutdown():
            self.lidar.subsets.show()
            self.right_wall = min(self.lidar.subsets.r3Array)
            self.left_wall = min(self.lidar.subsets.l3Array)
            self.front_wall = min(self.lidar.subsets.frontArray)
            # TODO: turn right is being called multiple times cos there isnt wall on the right of the thingy
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
        
        
        