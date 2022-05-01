#!/usr/bin/env python
"""
Image Based Visual Servoing
"""

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Pose2D, Twist
from rospy.numpy_msg import numpy_msg
from Point2D.msg import Point2D

class IBVSController():
    """
    Image Based Visual Servo Controller
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) # create publisher
        self._sub = rospy.Subscriber("ibvs_features", numpy_msg(Point2D), self.feature_update_received) # create subscriber

    def set_target(self, feature_coordinates : np.array) -> None:
        """
        Set the desired coordinates of the features of the target
        """
        self.target = feature_coordinates

    def set_target_depth(self, target_depth : float) -> None:
        """
        Set the desired depth (Z) of the target
        """
        self.target_depth = target_depth

    def set_tolerance(self, tolerance) -> None:
        """
        Set the tolerance below which the error is acceptable
        """
        self.tolerance = tolerance

    # def get_current_position(self) -> Pose:
    #     """
    #     get current 2D coordinates of the feature from camera. In case of IBVS the Pose contains a 2D
    #     """
    #     pass

    # def get_current_depth(self) -> float:
    #     """
    #     Return measured, observed or assumed depth (Z) of target
    #     """
    #     pass

    def feature_update_received(self, feature_coordinates : np.array) -> None:
        """
        Callback function for new features from camera
        """
        self.current_features = feature_coordinates # save current feature data 
        self._pub.publish(self.calc_velocities())   # calculate new velocities and publish the result 
        


    def calc_interaction_matrix(self) -> np.ndarray:
        """
        Calculate the interaction matrix L given the positions of 
        TODO include option to allow choosing between approximations of distance Z
        """

        (_, b) = np.shape(self.current_features) # check if there are at least 3 features
        if b < 3:
            raise ValueError('Feature vector is too short to calculate interaction matrix.')

        L = np.zeros(0,6)               # initialize empty matrix with 6 cols
        Z = self.target_depth           # assume Z to be target depth
        for feature in self.current_features: 
            [x, y] = feature
            L = np.append([[-1/Z, 0, x/Z, x*y, -(1+x*x), y], [0, -1/Z, y/Z, 1+y*y, -x*y, -x]])
        return L


    def calc_velocities(self, lambda_c = 1) -> Twist:
        """
        invert L_hat (MP Pseudoinverse) and calculate joint velocities
        """
        velocity_cmd = Twist()                      # initialize empty Twist message

        _L = self.calc_interaction_matrix()         
        _L_inv = np.linalg.pinv(_L)

        _e = self.current_features-self.target      # calc error [[dx1, dy1],  [dx2, dy2],  [dx3, dy3]]
        _e_flat = _e.flatten()                           # flaten error [dx1, dy1, dx2, dy2, dx3, dy3]

        _vel = - lambda_c * np.dot(_L_inv, _e_flat) # calculate velocity command vector

        # pack the Twist message
        velocity_cmd.linear.x = _vel[0]
        velocity_cmd.linear.y = _vel[1]
        velocity_cmd.linear.z = _vel[2]
        velocity_cmd.angular.x = _vel[3]
        velocity_cmd.angular.y = _vel[4]
        velocity_cmd.angular.z = _vel[5]

        return velocity_cmd
        


# Main function.
if __name__ == "__main__":
    rospy.init_node('vservo_node') # init ROS node named vservo_node
    controller = IBVSController()  # create controller instance

    ## Set desired camera depth and desired feature coordinates as well as distance from goal before stopping
    target_depth = 0.1
    target = np.array([10,10,-10,10,10,-10,-10,-10])
    tolerance = 0.5
    
    controller.calc_velocities()

    
    rospy.spin()