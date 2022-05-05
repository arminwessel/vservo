#!/usr/bin/env python3
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


class ArucoDetector():
    """
    Find corner positions of ArUCo markers
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        self._pub = rospy.Publisher("ibvs_features", Point2DArray, queue_size=1) # create publisher
        self._pub2 = rospy.Publisher("markers_image", Image, queue_size=20) # create publisher
        self._sub = rospy.Subscriber("usb_cam/image_raw", Image, self.image_received) # create subscriber
        self._type = "DICT_5X5_50"
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[self._type])
        self.arucoParams = cv2.aruco.DetectorParameters_create()


    def image_received(self, image_message : Image) -> None:
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        self.detect_markers(cv_image)

    def conv_Point2DArray_2_nparray(self, data : Point2DArray) -> np.array:
        try:
            _points = data.points
            _length = data.length
        except AttributeError:
            rospy.logerr('Argument passed to set_target does not have expected Attributes of type Point2DArray')
            return
        
        out = np.zeros([0,2])               # initialize empty matrix with 2 cols
        for point in _points:
            out=np.append(out,[[point.x, point.y]])
        return out

    def conv_nparray_2_Point2DArray(self, data: np.array) -> Point2DArray:
        out = Point2DArray()
        for idx, row in enumerate(data): # idx is indexing the elements in the loop starting with 0
            [_x,_y] = row
            point = Point2D()
            point.x ,point.y =_x, _y
            out.points.append(point)
        out.length=idx+1 # final value of idx is last index, add one to get number of iterations on for loop
        return out

    def detect_markers(self, frame):     # -> TODO:
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, self.arucoDict)
        frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        cv2.imwrite("./img.png", frame_markers)
        self._pub2.publish(self.bridge.cv2_to_imgmsg(frame_markers, encoding="rgb8"))
        
        if len(corners) > 0:
            ids = ids.flatten() 		# flatten the ArUco IDs list
        else:
            return

        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners_array = markerCorner.reshape((4, 2))
            corners = self.conv_nparray_2_Point2DArray(corners_array.astype(int))
            self._pub.publish(corners)
            
            # (topLeft, topRight, bottomRight, bottomLeft) = corners
            # # convert each of the (x, y)-coordinate pairs to integers
            # topRight = (int(topRight[0]), int(topRight[1]))
            # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            # topLeft = (int(topLeft[0]), int(topLeft[1]))    



# Main function.
if __name__ == "__main__":
    rospy.init_node('aruco_detector') # init ROS node named aruco_detector
    rospy.loginfo('#Node aruco_detector running#')
    arucodet = ArucoDetector()  # create instance
    
    rospy.spin()
