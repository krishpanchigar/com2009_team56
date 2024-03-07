#!/usr/bin/env python

import rospy
import time
from map_server.srv import SaveMap

def save_map_periodically():
    rospy.init_node('map_saver_periodic', anonymous=True)
    rospy.wait_for_service('static_map')
    save_map_service = rospy.ServiceProxy('static_map', SaveMap)

    rate = rospy.Rate(0.0167)  # once per minute
    while not rospy.is_shutdown():
        try:
            save_map_service("/task2/map/task2")
            rospy.loginfo("Map saved.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        rate.sleep()

if __name__ == "__main__":
    try:
        save_map_periodically()
    except rospy.ROSInterruptException:
        pass