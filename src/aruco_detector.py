#!/usr/bin/env python
"""
Node to find corners of ArUcCo marker
"""
import rospy
import numpy as np
from sensor_msgs.msg import Image
from vservo.msg import Point2D, Point2DArray
import cv2
from aruco_dict import ARUCO_DICT
from cv_bridge import CvBridge
import utils

class ArucoDetector():
    """
    Find corner positions of ArUCo markers
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        self._pub = rospy.Publisher("ibvs_features", Point2DArray, queue_size=1) # create publisher
        self._sub = rospy.Subscriber("camera_node/image_raw", Image, self.image_received) # create subscriber
        self._type = "DICT_5X5_50"
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[self._type])
        self.arucoParams = cv2.aruco.DetectorParameters_create()


    def image_received(self, image_message : Image) -> None:
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        self.detect_markers(cv_image)

    def detect_markers(self, frame) -> None:
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, self.arucoDict)     
        
        # check if no marker was detected
        if ids == None:
            empty = Point2DArray()
            self._pub.publish(empty) # publish empty Point2D message
            return
        elif len(ids)>1:
            empty = Point2DArray()
            self._pub.publish(empty) # publish empty Point2D message
            rospy.logwarn('More than one aruco tag detected')
            return

        # compose and publish ROS message to /ibvs_features
        corners_array = corners[0].reshape((4, 2)) # convert one element tuple to element, then reshape array
        detected_corners = utils.conv_nparray_2_Point2DArray(corners_array.astype(int))
        self._pub.publish(detected_corners)



# Main function.
if __name__ == "__main__":
    rospy.init_node('aruco_detector') # init ROS node named aruco_detector
    rospy.loginfo('#Node aruco_detector running#')
    arucodet = ArucoDetector()  # create instance
    
    rospy.spin()
