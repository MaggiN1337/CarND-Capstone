#!/usr/bin/env python
import timeit

import rospy
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane, Waypoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3


def distance(position1, position2):
    return math.sqrt((position1.x-position2.x)**2 + (position1.y-position2.y)**2 + (position1.z-position2.z)**2)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        self.has_image = True
        self.camera_image = msg
        # debug
        #start_processing = timeit.default_timer()
        light_wp, state = self.process_traffic_lights()
        # debug
        #rospy.logwarn("Image processing took {0} seconds".format(timeit.default_timer()-start_processing))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose, waypoint):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (position): position to match a waypoint to
            waypoint (waypoint): position of the waypoint

        Returns:
            int: index of the closest waypoint

        """
        min_dist = 9999
        index = -1

        for i in range(len(waypoint)):
            dist = distance(pose, waypoint[i].pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                index = i

        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        stop_line_pose = None
        stop_line_wp = None

        if self.pose and self.waypoints:
            # find the closest visible traffic light (if one exists)
            car_id = self.get_closest_waypoint(self.pose.pose.position, self.waypoints.waypoints)
            car_position = self.waypoints.waypoints[car_id].pose.pose.position

            light_id = self.get_closest_waypoint(car_position, self.lights)
            if light_id != -1:
                light_waypoint_id = self.get_closest_waypoint(self.lights[light_id].pose.pose.position,
                                                              self.waypoints.waypoints)
                light_position = self.waypoints.waypoints[light_waypoint_id].pose.pose.position
                #rospy.logwarn("Car Position: {0] - Next light: {1}".format(car_position, light_position))

                # if nearest waypoint in front of vehicle
                if light_waypoint_id > car_id:
                    dist_to_light = distance(car_position, light_position)

                    # if light is close enough to consider
                    if dist_to_light < 300:

                        # List of positions that correspond to the line to stop in front of for a given intersection
                        stop_line_positions = self.config['stop_line_positions']
                        stop_lines = []
                        for line_position in stop_line_positions:
                            wp = Waypoint()
                            wp.pose.pose.position.x = line_position[0]
                            wp.pose.pose.position.y = line_position[1]
                            wp.pose.pose.position.z = 0.0
                            stop_lines.append(wp)

                        # calc distance to closest stop line of the closest traffic light
                        light = self.lights[light_id]
                        stop_line_id = self.get_closest_waypoint(light_position, stop_lines)
                        stop_line_pose = stop_lines[stop_line_id].pose.pose
                        stop_line_wp = self.get_closest_waypoint(stop_line_pose.position, self.waypoints.waypoints)
                        #rospy.logwarn("Next traffic light stop line ahead: {0}".format(stop_line_wp))

        if light and stop_line_pose:
            state = self.get_light_state(light)
            return stop_line_wp, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
