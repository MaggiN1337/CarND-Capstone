#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
REFRESH_RATE = 10   # in Hz, set to 50 for carla
MAX_DECEL = .6
DISTANCE_TO_LANE = 5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb())

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_lane = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_id = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(REFRESH_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        # store vehicle position
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def get_closest_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        # multiply vectors
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        self.stopline_wp_id = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            #rospy.logwarn("Index wp1: {0}, Index wp2+1: {1}".format(wp1,wp2+1))
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # publish next n waypoints as a Lane
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_id()
        farthest_idx = closest_idx + LOOKAHEAD_WPS

        # debug to verify variables
        rospy.logwarn("stopline_id: {0} - closest_id: {1} - farthest_id: {2}"
                      .format(self.stopline_wp_id, closest_idx, farthest_idx))

        # if no stopline found or in range of LOOKAHEAD_WPS
        if self.stopline_wp_id == -1 or (self.stopline_wp_id >= farthest_idx):
            # select waypoints from closest waypoint up to LOOKAHEAD_WPS
            lane.waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        else:
            # select waypoints from closest waypoint up to stopline
            lane.waypoints = self.decelerate_waypoints(self.base_lane.waypoints[closest_idx:self.stopline_wp_id],
                                                       closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            #rospy.logwarn("i: {0}".format(i))
            p = Waypoint()
            p.pose = wp.pose

            # Two waypoints back from line so front of car stops at line
            stop_id = max(self.stopline_wp_id - closest_idx - DISTANCE_TO_LANE, 0)

            # calc distance to stop point
            dist = self.distance(waypoints, i, stop_id)

            # reduce velocity if distance gets smaller
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # at the end, come to stop smoothly
            if vel < 1.:
                vel = 0.

            # get the min. of calculated velocity and the speed limit
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
