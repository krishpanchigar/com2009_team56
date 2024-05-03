#!/usr/bin/env python3 

from distutils.log import error
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
        self.motion.publish_velocity()
        time.sleep(1)

    def moveForward(self):
        front_dist = min(self.lidar.subsets.frontArray)
        while front_dist > 0.4:
            print("Going forward")
            self.motion.set_velocity(self.fwd_vel, 0)
            self.motion.publish_velocity()
            front_dist = min(self.lidar.subsets.frontArray)
        self.stop()
        time.sleep(1)
        return

    def adjustToRightWall(self):
        right_wall_dist = min(self.lidar.subsets.r3Array)

        if right_wall_dist < self.right_wall_desired_dist - self.error:
            # if robot is close to the wall, turn left
            self.motion.set_velocity(0.0, self.ang_vel)
        elif right_wall_dist > self.right_wall_desired_dist + self.error:
            # if robot is far from right wall, turn right
            self.motion.set_velocity(0.0, -self.ang_vel)
        else:
            self.motion.set(self.fwd_vel, 0.0)
        
        self.motion.publish_velocity()

    def turn_90(self, direction):
        current_yaw = self.odom.yaw
        print(f"Current yaw: {current_yaw}")

        if direction == "right":
            
            target_yaw = current_yaw - 90
            ang_vel = -self.ang_vel
        elif direction == "left":
            target_yaw = current_yaw + 90
            ang_vel = self.ang_vel

        print(f"Target yaw: {target_yaw}")
        

        while abs(self.odom.yaw - target_yaw) > 1:
            self.odom.show()
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
    
    def correction(self):
        if min(self.lidar.subsets.r3Array) < 0.5 and min(self.lidar.subsets.l3Array) < 0.5:
            if self.right_wall > self.right_wall_control + self.error:
                self.motion.set_velocity(self.fwd_vel, -self.ang_vel)
                print('right')
            elif self.right_wall < self.right_wall_control - self.error:
                self.motion.set_velocity(self.fwd_vel, self.ang_vel)
                print('left')
            else:
                self.motion.set_velocity(self.fwd_vel, 0.0)
                print('fwd')
            

    def run(self):
        self.moveForward()
        while not rospy.is_shutdown():
            self.right_wall = min(self.lidar.subsets.r3Array)
            self.left_wall = min(self.lidar.subsets.l3Array)
            self.front_wall = min(self.lidar.subsets.frontArray)
            # TODO: turn right is being called multiple times cos there isnt wall on the right of the thingy
            # TODO: turn 90 can be improved by using odom data better somehow
            # if there is no wall on the right, turn right
            if self.right_wall > 0.4:
                self.stop()
                self.turn_90("right")
                print("Turning right")
            # if there is wall in front and a wall on the right, turn left
            elif self.front_wall < 0.4:
                self.turn_90("left")
                print("Turning left")
            # if there is no wall in the front, and there is a right wall, keep going forward
            else:
                self.moveForward()

            
        
if __name__ == '__main__':
    rospy.init_node('maze_follower')
    maze_follower = MazeFollower()
    maze_follower.run()
    rospy.spin()
    rospy.on_shutdown((maze_follower.motion.stop(), maze_follower.motion.publish_velocity()))
        
        
        