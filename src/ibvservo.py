#!/usr/bin/env python3
"""
Image Based Visual Servoing
"""

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Pose2D, Twist
from vservo.msg import Point2D, Point2DArray

class IBVSController():
    """
    Image Based Visual Servo Controller
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) # create publisher
        self._sub = rospy.Subscriber("ibvs_features", Point2DArray, self.feature_update_received) # create subscriber
        _array_current_fearures =np.array([[-11,-11],[-11,11],[11,11],[11,-11]])
        self.current_features = self.conv_nparray_2_Point2DArray(_array_current_fearures)


    def set_target(self, feature_coordinates : Point2DArray) -> None:
        """
        Set the desired coordinates of the features of the target
        """
        try:
            _points = feature_coordinates.points
            _length = feature_coordinates.length
        except AttributeError:
            rospy.logerr('Argument passed to set_target does not have expected Attributes of type Point2DArray')
            return
        if _length < 3:
            raise ValueError('Target length is less than 3, the minimum required is 3')
        self.target = feature_coordinates
    
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
    
    def calc_error(self) -> np.array:
        _array_current = self.conv_Point2DArray_2_nparray(self.current_features)
        _array_target = self.conv_Point2DArray_2_nparray(self.target)
        return _array_target-_array_current


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

    def feature_update_received(self, feature_coordinates : Point2DArray) -> None:
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

        # check if there are at least 3 features
        if self.current_features.length < 3:
            raise ValueError('Feature vector is too short to calculate interaction matrix.')

        L = np.zeros([0,6])               # initialize empty matrix with 6 cols
        Z = self.target_depth           # assume Z to be target depth

        for point in self.current_features.points: 
            x, y = point.x, point.y
            L = np.append(L,[[-1/Z, 0, x/Z, x*y, -(1+x*x), y], [0, -1/Z, y/Z, 1+y*y, -x*y, -x]],axis=0)
        return L

    def calc_velocities(self, lambda_c = 1) -> Twist:
        """
        invert L_hat (MP Pseudoinverse) and calculate joint velocities
        """
        velocity_cmd = Twist()                      # initialize empty Twist message

        _L = self.calc_interaction_matrix()         
        _L_inv = np.linalg.pinv(_L)

        _e = self.calc_error()     # calc error [[dx1, dy1],  [dx2, dy2],  [dx3, dy3]]
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
    rospy.loginfo('#Node vservo_node running#')
    controller = IBVSController()  # create controller instance

    ## Set desired camera depth and desired feature coordinates as well as distance from goal before stopping
    controller.set_target_depth(0.2)
    targetarray = np.array([[-10,-10],[-10,10],[10,10],[10,-10]])
    controller.set_target(controller.conv_nparray_2_Point2DArray(targetarray))
    controller.set_tolerance(0.05)

    
    rospy.spin()