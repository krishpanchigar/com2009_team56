#!/usr/bin/env python3

import rospy
import math
import time
import numpy as np
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
import waffle

class WallFollowerPID:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.motion = waffle.Motion(debug=True)
        self.lidar = waffle.Lidar(debug=True)
        self.odom = waffle.Pose(debug=True)
        
        self.P = 5.5 #6
        self.I = 0
        self.D = 10
        
        self.desired_distance = 0.3
        self.prev_error = 0.0
        self.integral = 0.0

        self.fwd_vel = 0.1 #0.2

    def stop(self):
        self.motion.set_velocity(0.0, 0.0)
        self.motion.publish_velocity()
        self.motion.stop()
        time.sleep(1)

    def pid_control(self, current_distance):
        error = self.desired_distance - current_distance
        
        self.integral += error
        
        derivative = error - self.prev_error
        
        correction = self.P * error + self.I * self.integral + self.D * derivative
        
        self.prev_error = error
        
        return correction

    def run(self):
        while not rospy.is_shutdown():
            right_distances = self.lidar.subsets.r3Array
            right_distance = min(right_distances)

            front_distances = self.lidar.subsets.frontArray
            front_distance = min(front_distances)
            
            if front_distance < 0.3:
                self.motion.set_velocity(0.0, 0.5) 
                self.motion.publish_velocity()
                rospy.loginfo("Turning left due to front wall")
                time.sleep(1.5) 
                self.stop()
            else:
                correction = self.pid_control(right_distance)
                
                self.motion.set_velocity(self.fwd_vel, correction)
                self.motion.publish_velocity()
            
            rospy.loginfo(f"Right distance: {right_distance}, Correction: {correction}")
            
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower_pid')
    wall_follower = WallFollowerPID()
    try:
        wall_follower.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        wall_follower.stop()
