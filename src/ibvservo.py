#!/usr/bin/env python
"""
Image Based Visual Servoing
"""

from attr import has
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Pose2D, Twist
from vservo.msg import Point2D, Point2DArray
import utils

class IBVSController():
    """
    Image Based Visual Servo Controller
    """

    def __init__(self) -> None:
        """
        Constructor
        """
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) # create publisher
        self._pub2 = rospy.Publisher("ibvs_target", Point2DArray, queue_size=1) # create publisher
        self._pub3 = rospy.Publisher("control_error", Point2DArray, queue_size=1) # create publisher
        self._sub = rospy.Subscriber("ibvs_features", Point2DArray, self.feature_update_received) # create subscriber
        center_target = utils.tf_cc(np.array([[40,40],[-40,40],[-40,-40],[40,-40]]))
        self.set_target(utils.conv_nparray_2_Point2DArray(center_target))

    def set_target(self, target_coordinates : Point2DArray) -> None:
        if target_coordinates == None:
            return
        self.target = target_coordinates

    
    def calc_error(self) -> np.array:
        if (not hasattr(self.current_features,'points')) or (not hasattr(self.target, 'points')):
            return np.zeros((4,2)) # return zero error if target or features are not defined
        if len(self.current_features.points) == 0:
            return np.zeros((4,2)) # return zero error if target or features are not defined
        _array_current = utils.conv_Point2DArray_2_nparray(self.current_features)
        _array_target = utils.conv_Point2DArray_2_nparray(self.target)
        error = _array_target-_array_current
        self._pub3.publish(utils.conv_nparray_2_Point2DArray(error))
        return error


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
        self._pub2.publish(self.target) # publish the current target
        

    def get_Z(self) -> float:
        if hasattr(self, 'target_depth'):
            return self.target_depth           # assume Z to be target depth
        else:
            rospy.logwarn('Assumed 0.1 for feature distance Z')
            return 0.1 
        
    def update_interaction_matrix(self) -> None:
        """
        Calculate the interaction matrix L given the positions of 
        TODO include option to allow choosing between approximations of distance Z
        """

        if hasattr(self.current_features, 'points'):
            if len(self.current_features.points)>0:
                L = np.zeros([0,6])               # initialize empty matrix with 6 cols
                Z = self.get_Z()
                for point in self.current_features.points: 
                    x, y = point.x, point.y
                    L = np.append(L,[[-1/Z, 0, x/Z, x*y, -(1+x*x), y], [0, -1/Z, y/Z, 1+y*y, -x*y, -x]],axis=0)
                self.L = L


    def calc_velocities(self, lambda_c = 1) -> Twist:
        """
        invert L_hat (MP Pseudoinverse) and calculate joint velocities
        """
        velocity_cmd = Twist()                      # initialize empty Twist message

        self.update_interaction_matrix()

        if not hasattr(self, 'L'): # If no interaction matrix is defined, set command velocity to all zeros
            _vel=np.zeros(6)
        else:
            _L_inv = np.linalg.pinv(self.L)

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
    controller.set_tolerance(0.05)

    
    rospy.spin()