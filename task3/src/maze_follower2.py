#!/usr/bin/env python3

import rospy
import math
import time
import numpy as np
import waffle

class WallFollowerPID:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.motion = waffle.Motion(debug=True)
        self.lidar = waffle.Lidar(debug=True)
        self.odom = waffle.Pose(debug=True)
        
        self.P = 1.5
        self.I = 0.0
        self.D = 2.1
        
        self.desired_distance = 0.3
        self.prev_error = 0.0
        self.integral = 0.0

        self.fwd_vel = 0.1 #0.2

    def stop(self):
        self.motion.set_velocity(0.0, 0.0)
        self.motion.publish_velocity()
        self.motion.stop()
        time.sleep(1)

    def calculate_forward_velocity(self):
        # TODO: improve fwd veolocity control
        front_distance = min(self.lidar.subsets.frontArray)
        right_distance = min(self.lidar.subsets.r3Array)

        if front_distance < 0.5:
            return 0.15
        else:
            return 0.25
        
    def right_wall_gap(self):
        time.sleep(0.5)
        self.motion.set_velocity(0, -math.pi/2)
        self.motion.publish_velocity()
        rospy.loginfo("Turning right due to gap in right wall")
        time.sleep(1.2) 
        self.stop()
        self.motion.set_velocity(0.75, 0)
        self.motion.publish_velocity()
        time.sleep(1.5) 
        self.stop()


    def pid_control(self):
        left_distance = min(self.lidar.subsets.l3Array)
        right_distance = min(self.lidar.subsets.r3Array)

        if right_distance > 0.75:
            current_distance = left_distance
            correction_dir = -1
            print("Using left wall")
        else:
            current_distance = right_distance
            correction_dir = 1
        
        if current_distance > 0.5 and correction_dir == -1:
            current_distance = 0.4
        
        error = self.desired_distance - current_distance

        # reseting integral when error crosses 0,
        if self.prev_error * error < 0:
            self.integral = 0
        
        self.integral += error

        max_integral = 1.0
        min_integral = -1.0
        self.integral = max(min(self.integral, max_integral), min_integral)
        
        derivative = error - self.prev_error
        
        correction = self.P * error + self.I * self.integral + self.D * derivative
        
        self.prev_error = error
        
        return correction * correction_dir

    def run(self):
        while not rospy.is_shutdown():
            right_distances = self.lidar.subsets.r3Array
            right_distance = min(right_distances)

            left_distance = min(self.lidar.subsets.l3Array)

            front_distances = self.lidar.subsets.frontArray
            front_distance = min(front_distances)

            back_distances = self.lidar.subsets.backArray
            back_distance = min(back_distances)

            self.fwd_vel = self.calculate_forward_velocity()

            # TODO: fix turning logic to follow the right-wall
            
            if front_distance < 0.35:
                if back_distance < 0.2:
                    rospy.loginfo("BACK ISSUE")
                    self.motion.set_velocity(0.5, 0)
                    self.motion.publish_velocity()
                    time.sleep(0.15)
                    self.motion.stop()
                if left_distance > right_distance:
                    if back_distance < 0.2 or right_distance < 0.2:
                        rospy.loginfo("BACK ISSUE")
                        self.motion.set_velocity(0.5, 0)
                        self.motion.publish_velocity()
                        time.sleep(0.15)
                    self.motion.set_velocity(0, 1.0)
                    self.motion.publish_velocity()
                    rospy.loginfo("Turning left due to front wall")
                else:
                    if back_distance < 0.2 or left_distance < 0.2:
                        rospy.loginfo("BACK ISSUE")
                        self.motion.set_velocity(0.5, 0)
                        self.motion.publish_velocity()
                        time.sleep(0.15)
                    self.motion.set_velocity(0, -1.0)
                    self.motion.publish_velocity()
                    rospy.loginfo("Turning right due to fornt wall")
                time.sleep(0.75) 
                self.stop()
            else:
                if right_distance > 0.6 and left_distance < 0.5:
                    right_distance = left_distance
                correction = self.pid_control()
                
                self.motion.set_velocity(self.fwd_vel, correction)
                self.motion.publish_velocity()
            
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
