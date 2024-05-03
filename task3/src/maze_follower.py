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
        self.stop()
        time.sleep(1)

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

        if direction == "right":
            target_yaw = current_yaw - math.pi / 2
            ang_vel = -self.ang_vel
        elif direction == "left":
            target_yaw = current_yaw + math.pi / 2
            ang_vel = self.ang_vel

        # normalising the target value between -pi and pi
        while target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        while target_yaw < -math.pi:
            target_yaw += 2 * math.pi

        while abs(self.odom.yaw - target_yaw) > 0.01:
            self.motion.set_velocity(0.0, ang_vel)
            self.motion.publish_velocity()
            self.rate.sleep()
        
        self.motion.stop()
        self.motion.publish_velocity()

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
        self.motion.stop()
        self.motion.publish_velocity()
    
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
        self.motion.set_velocity(self.fwd_vel, 0.0)
        self.motion.publish_velocity()
        self.left_wall_control = min(self.lidar.subsets.l3Array)
        self.right_wall_control = min(self.lidar.subsets.r3Array)
        # Following right wall
        while not rospy.is_shutdown():
            self.left_wall = min(self.lidar.subsets.l3Array)
            self.right_wall = min(self.lidar.subsets.r3Array)
            
            # Keeping the turtle within the error
            self.correction()
            
            if min(self.lidar.subsets.frontArray) < 0.4:
                #Check for where walls are:
                self.motion.stop()
                self.motion.publish_velocity()
                
                if min(self.lidar.subsets.r3Array) > 0.4:
                    print('No right wall detected. Turning right')
                    self.turn(-self.ang_vel)
                else:
                    print('Right wall detected')
                    if min(self.lidar.subsets.l3Array) > 0.4:
                        print('No left wall detected. Turning left')
                        self.turn(self.ang_vel) 
                    else:
                        print('Left wall detected')
                        print('Dead end')
                        self.motion.set_velocity(0.0, -self.ang_vel)
                        self.motion.publish_velocity()
                        time.sleep(2*self.turn_time)
                        break
                
                self.motion.stop()
                self.motion.publish_velocity()
            
        
if __name__ == '__main__':
    rospy.init_node('maze_follower')
    maze_follower = MazeFollower()
    maze_follower.run()
    rospy.spin()
    rospy.on_shutdown((maze_follower.motion.stop(), maze_follower.motion.publish_velocity()))
        
        
        