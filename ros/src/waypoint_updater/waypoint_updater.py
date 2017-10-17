#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers for /current_pose and /vase_waypoints:
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # subscriber for /traffic_waypoint and /obstacle_waypoint:
        # rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb, queue_size=1)

        # Publisher in final_waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables
        self.sampling_rate = 10.
        self.car_pose = None
        self.car_orientation = None
        self.waypoints = []
        self.closest_waypoint = None
        self.final_waypoints = []
        self.process_info()

        rospy.spin()

    def pose_cb(self, msg):
        self.car_pose = msg.pose

    def waypoints_cb(self, msg):
        self.waypoints = [waypoint for waypoint in msg.waypoints]
        self.base_waypoints_sub.unregister()

        # we need to do this action just one time
        rospy.loginfo("Unregistered from /base_waypoints topic")

    def get_final_waypoints(self, car_pose, waypoints):
        self.closest_waypoint = self.get_closest_waypoint(car_pose.position, waypoints)
        final_wyp = []
        for idx in range(self.closest_waypoint, len(waypoints)):
            final_wyp.append(waypoints[idx])
            if len(final_wyp) > LOOKAHEAD_WPS:
                break
        return final_wyp

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, current_position, waypoints):
        closer_dist = None
        closer_idx = None
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for idx in range(0, len(waypoints)):
            dist = dl(current_position, waypoints[idx].pose.pose.position)
            if closer_dist is None or dist < closer_dist:
                closer_dist = dist
                closer_idx = idx
        return closer_idx

    def process_info(self):
        rate = rospy.Rate(self.sampling_rate)
        while not rospy.is_shutdown():
            if self.waypoints is not None and self.car_pose is not None:
                self.final_waypoints = self.get_final_waypoints(self.car_pose, self.waypoints)
                # create msg with the final waypoints:
                final_waypoints_msg = Lane()
                final_waypoints_msg.waypoints = list(self.final_waypoints)
                self.final_waypoints_pub.publish(final_waypoints_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
