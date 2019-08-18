#!/usr/bin/env python
import timeit

import rospy
import math

from scipy.spatial import KDTree
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

from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

STATE_COUNT_THRESHOLD = 3


def distance(position1, position2):
    return math.sqrt(
        (position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2 + (position1.z - position2.z) ** 2)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

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

        # YOLO for ROS message
        sub5 = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_cb)
        self.BoundingBox_List = None

        self.simulator_mode = rospy.get_param("/simulator_mode")
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

        self.lights = []
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                 for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

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
        light_wp, state = self.process_traffic_lights()
        # debug
        # rospy.logwarn("Next traffic light stop line ahead: {0} - State: ".format(light_wp, state))
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

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (position): position to match a waypoint to

        Returns:
            int: index of the closest waypoint

        """
        # use KDTree
        index = self.waypoint_tree.query([x, y], 1)[1]

        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # Tdebug: return light state from simulator to test without image recognition
        # return light.state

        if not self.has_image:
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image, self.BoundingBox_List, 1)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        closest_light = None
        line_wp_id = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_wp_id = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_id = self.get_closest_waypoint(line[0], line[1])
                # find closest stop line waypoint index
                d = temp_wp_id - car_wp_id
                if 0 <= d < diff:
                    diff = d
                    closest_light = light
                    line_wp_id = temp_wp_id

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_id, state

        return -1, TrafficLight.UNKNOWN

    def bounding_boxes_cb(self, msg):
        # create new list of bounding boxes
        self.BoundingBox_List = []

        if int(self.simulator_mode) == 1:
            expected_probability = 0.85
            diagonal_size = 85
        else:
            expected_probability = 0.25
            diagonal_size = 40

        for boundingBox in msg.bounding_boxes:

            if str(boundingBox.Class) == 'traffic light' and boundingBox.probability >= expected_probability:

                # if image_size aka boundingBox size is big enough
                if math.sqrt((boundingBox.xmin - boundingBox.xmax) ** 2 + (
                        boundingBox.ymin - boundingBox.ymax) ** 2) >= diagonal_size:
                    self.BoundingBox_List.append(boundingBox)

                    # park site mode
                    if int(self.simulator_mode) == 0:
                        # get camera image
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                        # store bounding box as image
                        bounding_box_image = cv_image[boundingBox.ymin:boundingBox.ymax, boundingBox.xmin:boundingBox.xmax]
                        self.light_classifier.detect_light_state(bounding_box_image)


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
