#!/usr/bin/env python3 

import rospy
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

class MazeFollower(object):
    def __init__(self):
        self.tb3_move = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_laser = Tb3LaserScan()
        self.rate = rospy.Rate(10) # Hz
        self.left_wall_control = 0.0
        self.right_wall_control = 0.0
        self.error = 0.08
        self.fwd_vel = 0.15
        self.ang_vel = 0.5

    def run(self):
        self.tb3_move.set_move_cmd(self.fwd_vel, 0.0)
        self.tb3_move.publish()
        self.left_wall_control = self.tb3_laser.left_wall
        self.right_wall_control = self.tb3_laser.right_wall
        # Following right wall
        while not rospy.is_shutdown():
            self.left_wall = self.tb3_laser.left_wall
            self.right_wall = self.tb3_laser.right_wall
            
            # Keeping the turtle within the error
            while self.tb3_laser.closest_object_position > 0.3 or self.tb3_laser.closest_object_position == 0.0:
                if self.right_wall > self.right_wall_control + self.error:
                    self.tb3_move.set_move_cmd(self.fwd_vel, -self.ang_vel)
                    print('right')
                elif self.right_wall < self.right_wall_control - self.error:
                    self.tb3_move.set_move_cmd(self.fwd_vel, self.ang_vel)
                    print('left')
                else:
                    self.tb3_move.set_move_cmd(self.fwd_vel, 0.0)
                    print('fwd')
                self.tb3_move.publish()
            
            print(self.tb3_laser.closest_object_position)
            
if __name__ == '__main__':
    rospy.init_node('maze_follower')
    maze_follower = MazeFollower()
    maze_follower.run()
    rospy.spin()
        
        
        