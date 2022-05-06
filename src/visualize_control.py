#!/usr/bin/env python3
"""
Node to Visualize controller
"""
from attr import has
import rospy
import numpy as np
from sensor_msgs.msg import Image
from vservo.msg import Point2D, Point2DArray
import cv2
from aruco_dict import ARUCO_DICT
from cv_bridge import CvBridge
import utils
from geometry_msgs.msg import Twist

class ControlVisualizer():
    """
    Visualize Control
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        self._sub = rospy.Subscriber("usb_cam/image_raw", Image, self.image_received)
        self._sub2 = rospy.Subscriber("ibvs_target", Point2DArray, self.target_update_received)
        self._sub3 = rospy.Subscriber("ibvs_features", Point2DArray, self.feature_update_received) 
        # self._sub4 = rospy.Subscriber("cmd_vel",Twist, self.command_update_received)
        self._pub = rospy.Publisher("image_control_overlay", Image, queue_size=20)
        self.bridge = CvBridge()
        return

    def target_update_received(self, data) -> None:
        self.target_points = utils.conv_Point2DArray_2_nparray(data)
        return

    def feature_update_received(self, data) -> None:
        self.feature_points = utils.conv_Point2DArray_2_nparray(data)
        return
    
    # def command_update_received(self, data) -> None:
    #     self.cmd_vel = data
    #     return

    def image_received(self, image_message : Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='rgb8') # convert ROS image to opencv
        self.frame = frame.copy() # work on copy
        self.overlay_image()
    
    def overlay_image(self) -> None:
        # if not hasattr(self, 'frame'):
        #     return
        # overlay a red box where the target is 
        if hasattr(self, 'target_points'):
            cv2.polylines(self.frame, 
                [self.target_points],
                isClosed = True,
                color = (255,0,0), # red
                thickness = 2)
        # else: 
        #     rospy.loginfo('No target advertised')
        
        # overlay green box on current feature
        if hasattr(self, 'feature_points'):
            cv2.polylines(self.frame, 
                [self.feature_points],
                isClosed = True,
                color = (0,255,0), # green
                thickness = 2)
        # else: 
        #     rospy.loginfo('No feature advertised')
        
        # publish
        self._pub.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="rgb8"))


# Main function.
if __name__ == "__main__":
    rospy.init_node('visualize_control') # init ROS node named visualize_control
    rospy.loginfo('#Node visualize_control running#')
    vizualizer = ControlVisualizer()  # create instance
    
    rospy.spin()
