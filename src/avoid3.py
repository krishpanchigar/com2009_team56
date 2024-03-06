import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan, Tb3Map
import numpy as np

class ExploreEnvironment:
    def __init__(self):
        rospy.init_node('explore_environment')

        # Initialize publishers and subscribers
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        self.tb3_map = Tb3Map()
        print("1")

        # Initialize SLAM
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.tb3_lidar.laserscan_cb)
        # Initialize variables
        self.forward_velocity = 0.15  # Forward velocity
        self.turn_velocity = 0.5  # Turning velocity
        self.distance_threshold = 0.25  # Distance threshold for obstacle avoidance
        print("3")

    
    def scan_callback(self, scan_msg):
        # Implement obstacle avoidance logic
        # Could use as a callback for the subscriber
        min_range = min(scan_msg.ranges)
        if min_range < self.distance_threshold:
            # If obstacle detected, turn
            self.turn_robot()

    def turn_robot(self):
        # Turn the robot
        self.robot_controller.set_move_cmd(angular=self.turn_velocity)
        self.robot_controller.publish()

    def calculate_ideal_path(self):
        # Get the current map data
        map_data = self.tb3_map.map_data

        # Get the current position and orientation of the robot
        posx = self.robot_odom.posx
        posy = self.robot_odom.posy
        yaw = self.robot_odom.yaw

        # Find the nearest unexplored area in the map
        unexplored_areas = np.where(map_data.data == -1)
        distances = np.sqrt((unexplored_areas[0] - posx) ** 2 + (unexplored_areas[1] - posy) ** 2)
        nearest_unexplored_area = unexplored_areas[np.argmin(distances)]

        # Calculate the direction to the nearest unexplored area
        direction_to_unexplored_area = np.arctan2(nearest_unexplored_area[1] - posy, nearest_unexplored_area[0] - posx)

        # Calculate the angular velocity needed to turn towards the unexplored area
        angular_velocity = direction_to_unexplored_area - yaw
        if angular_velocity > np.pi:
            angular_velocity -= 2 * np.pi
        elif angular_velocity < -np.pi:
            angular_velocity += 2 * np.pi

        # Set the move command to turn towards the unexplored area
        self.robot_controller.set_move_cmd(angular=angular_velocity)


    def explore(self):
        # Main exploration loop
        rate = rospy.Rate(10)  # 10 Hz
        print("4")
        while not rospy.is_shutdown():
            print("2")
            # Follow the ideal path
            self.calculate_ideal_path()
            self.robot_controller.set_move_cmd(linear=self.forward_velocity)
            self.robot_controller.publish()
            rate.sleep()
            

if __name__ == '__main__':
    explore = ExploreEnvironment()
    print("5")
    explore.explore()