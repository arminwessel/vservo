#!/usr/bin/env python3
"""
Node to find corners of ArUco marker
"""
import rospy
import numpy as np
from vservo.msg import Point2D, Point2DArray

class ArucoExtractor():
    """
    Simulation for features found in camera image
    using random integers
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        self._pub = rospy.Publisher("ibvs_features", Point2DArray, queue_size=1) # create publisher
        self._sub = rospy.Subscriber("poke", Int8, self.poke_received) # create subscriber
