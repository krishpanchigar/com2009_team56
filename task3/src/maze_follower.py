#!/usr/bin/env python3 

import rospy
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
import waffle
import math
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
        self.ang_vel = 0.5

    def stop(self):
        self.motion.stop()

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
            while min(self.lidar.subsets.frontArray) > 0.3 or min(self.lidar.subsets.r3Array) < 0.3 or  math.isnan(self.lidar.subsets.front):
                if self.right_wall > self.right_wall_control + self.error:
                    self.motion.set_velocity(self.fwd_vel, -self.ang_vel)
                    print('right')
                elif self.right_wall < self.right_wall_control - self.error:
                    self.motion.set_velocity(self.fwd_vel, self.ang_vel)
                    print('left')
                else:
                    self.motion.set_velocity(self.fwd_vel, 0.0)
                    print('fwd')
                self.motion.publish_velocity()
            
            #Check for where walls are:
            self.motion.stop()
            self.motion.publish_velocity()
            
            if min(self.lidar.subsets.r3Array) > 0.3:
                print('No right wall detected. Turning right')
            else:
                print('Right wall detected')
                if min(self.lidar.subsets.l3Array) > 0.3:
                    print('No left wall detected. Turning left')
                else:
                    print('Left wall detected')
                    print('Dead end')
                    break
                
            
            print(min(self.lidar.subsets.frontArray))
            
if __name__ == '__main__':
    rospy.init_node('maze_follower')
    maze_follower = MazeFollower()
    maze_follower.run()
    rospy.spin()
    rospy.on_shutdown(maze_follower.stop)
        
        
        