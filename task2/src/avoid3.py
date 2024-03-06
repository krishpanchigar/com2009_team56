import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan, Tb3Map
from slam_gmapping import SlamGMapping  # Import the SlamGMapping class
import numpy as np

class ExploreEnvironment:
    def __init__(self):
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
        self.robot_map = Tb3Map()
        self.map_data = OccupancyGrid()
        self.gmapping = SlamGMapping()  # Create an instance of the SlamGMapping class

    def scan_callback(self, scan_msg):
        self.robot_scan.laserscan_cb(scan_msg)

    def turn_robot(self):
        yaw = self.robot_odom.yaw
        unexplored_areas = np.where(self.map_data.data == -1)
        distances = np.sqrt((unexplored_areas[0] - self.robot_odom.posx) ** 2 + (unexplored_areas[1] - self.robot_odom.posy) ** 2)
        nearest_unexplored_area = unexplored_areas[np.argmin(distances)]
        direction_to_unexplored_area = np.arctan2(nearest_unexplored_area[1] - self.robot_odom.posy, nearest_unexplored_area[0] - self.robot_odom.posx)
        angular_velocity = direction_to_unexplored_area - yaw
        self.robot_controller.set_move_cmd(angular=angular_velocity)

    def move_robot(self):
        if self.robot_scan.min_distance < 0.5:
            self.robot_controller.set_move_cmd(linear=0.0, angular=0.0)
        else:
            self.robot_controller.set_move_cmd(linear=0.2, angular=0.0)
        self.robot_controller.publish()

    def explore(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.move_robot()
            self.turn_robot()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('explore_environment')
    explore = ExploreEnvironment()
    explore.explore()