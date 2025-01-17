import cv2
import numpy as np
from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge


class TLClassifier(object):
    def __init__(self):
        # load classifier
        self.bridge = CvBridge()

    # Return state of light
    def detect_light_state(self, bb_image):
        # Get height and width
        height, width, channels = bb_image.shape
        # split into Red, Yellow and Green Areas
        red_area = bb_image[0:height // 3, 0:width]
        yellow_area = bb_image[height // 3: 2 * height // 3, 0:width]
        green_area = bb_image[2 * height // 3: height, 0:width]

        # coefficients for BGR range
        # Standard Gray conversion, coefficients = [0.114, 0.587, 0.299] (bgr)
        coefficients_red = [0.1, 0.1, 0.8]
        coefficients_yellow = [0.114, 0.587, 0.299]
        coefficients_green = [0.1, 0.8, 0.1]

        # Apply coefficients to get grayscale image
        red_area = cv2.transform(red_area, np.array(coefficients_red).reshape((1, 3)))
        yellow_area = cv2.transform(yellow_area, np.array(coefficients_yellow).reshape((1, 3)))
        green_area = cv2.transform(green_area, np.array(coefficients_green).reshape((1, 3)))

        # merge the greyscale image back
        bb_image = np.concatenate((red_area, yellow_area, green_area), axis=0)

        # Get height and width values again
        height, width = bb_image.shape

        # Create mask
        mask = np.zeros((height, width), np.uint8)
        # Can play around with offset
        width_offset = 3
        height_offset = 4
        cv2.ellipse(mask, (width // 2, 1 * height // 6), (width // 2 - width_offset, height // 6 - height_offset), 0, 0,
                    360, 1, -1)
        cv2.ellipse(mask, (width // 2, 3 * height // 6), (width // 2 - width_offset, height // 6 - height_offset), 0, 0,
                    360, 1, -1)
        cv2.ellipse(mask, (width // 2, 5 * height // 6), (width // 2 - width_offset, height // 6 - height_offset), 0, 0,
                    360, 1, -1)

        # Apply mask
        bb_image = np.multiply(bb_image, mask)

        # Threshold the grayscale image
        bb_image = cv2.inRange(bb_image, 210, 255)

        # Partition into Red, Yellow and Green Areas
        red_area = bb_image[0:height // 3, 0:width]
        yellow_area = bb_image[height // 3: 2 * height // 3, 0:width]
        green_area = bb_image[2 * height // 3: height, 0:width]
        # Count the number of non-zero pixels
        red_count = cv2.countNonZero(red_area)
        yellow_count = cv2.countNonZero(yellow_area)
        green_count = cv2.countNonZero(green_area)

        # Default state is unknown
        state = TrafficLight.UNKNOWN
        # Determine which color had max non-zero pixels
        if red_count > yellow_count and red_count > green_count:
            state = TrafficLight.RED
        elif yellow_count > red_count and yellow_count > green_count:
            state = TrafficLight.YELLOW
        elif green_count > red_count and green_count > yellow_count:
            state = TrafficLight.GREEN
        else:
            rospy.logwarn("No traffic light color recognized")

        return state

    def get_classification(self, image, bounding_box_list, simulator_mode):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
            bounding_box_list (List): List containing bounding boxe(s) of Traffic Lights
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            :param image:
            :param bounding_box_list:
            :param simulator_mode:
        """
        # if list is empty, return UNKNOWN
        if not bounding_box_list:
            return TrafficLight.UNKNOWN

        else:
            # take the boundingBox with highest probability (at index 0)
            # TODO: only take BBs that are for  current lane
            xmin = bounding_box_list[0].xmin
            xmax = bounding_box_list[0].xmax
            ymin = bounding_box_list[0].ymin
            ymax = bounding_box_list[0].ymax

            # cutout bounding box from image
            bb_image = image[ymin:ymax, xmin:xmax]

            # Check if running in simulator mode
            if int(simulator_mode) == 1:

                # Convert to HSV
                hsv_bb_img = cv2.cvtColor(bb_image, cv2.COLOR_BGR2HSV)

                # Red Color ranges (Red has two ranges)
                frame_threshed_red1 = cv2.inRange(hsv_bb_img, (0, 70, 50), (10, 255, 255))
                frame_threshed_red2 = cv2.inRange(hsv_bb_img, (170, 70, 50), (180, 255, 255))

                # Yellow Color range
                frame_threshed_yellow = cv2.inRange(hsv_bb_img, (40.0 / 360 * 255, 100, 100),
                                                    (66.0 / 360 * 255, 255, 255))
                # Green color range
                frame_threshed_green = cv2.inRange(hsv_bb_img, (90.0 / 360 * 255, 100, 100),
                                                   (140.0 / 360 * 255, 255, 255))

                # If more than a certain number of pixels are red
                if cv2.countNonZero(frame_threshed_red1) + cv2.countNonZero(frame_threshed_red2) > 40:
                    return TrafficLight.RED
                elif cv2.countNonZero(frame_threshed_yellow) > 20:
                    return TrafficLight.YELLOW
                elif cv2.countNonZero(frame_threshed_green) > 20:
                    return TrafficLight.GREEN
                else:
                    rospy.logwarn("No traffic light color recognized")
                    return TrafficLight.UNKNOWN

            # Running in site mode
            else:
                return self.detect_light_state(bb_image)
