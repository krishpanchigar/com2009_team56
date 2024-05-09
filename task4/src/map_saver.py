#!/usr/bin/env python3

import roslaunch
import rospy

file_path = "/home/student/catkin_ws/src/com2009_team56/maps/task4_map"

rospy.init_node("map saver node", anonymous=True)
rate = rospy.Rate(0.5)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

while not rospy.is_shutdown():
    print(f"Saving map file at time: {rospy.get_time()}...")
    node = roslaunch.core.Node(
        package="map_server",
        node_type="map_saver",
        args=f"-f {file_path}",
        output="screen"
    )
    process = launch.launch(node)
    rospy.sleep(3)
