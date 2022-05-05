#!/usr/bin/env python3
"""
Node to simulate finding features in image
"""
import rospy
import numpy as np
from std_msgs.msg import Int8
from vservo.msg import Point2D, Point2DArray

class RandomFeatureGenerator():
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
    
    # TODO find a way to reuse methods over several objects - without inheritance

    def poke_received(self, arg) -> None:
        rospy.loginfo('# poke received')
        _random_array = np.random.random_integers(-30, 30, 8) # returns 8 random integers between -30 and 30 
        _random_array = _random_array.reshape((4, 2)) # reshape the array into the form [[1,2],[3,4],[5,6],[7,8]]
        data = self.conv_nparray_2_Point2DArray(_random_array)
        self._pub.publish(data)



# Main function.
if __name__ == "__main__":
    rospy.init_node('random_feature_generator') # init ROS node named random_feature_generator
    rospy.loginfo('#Node random_feature_generator running#')
    featuregen = RandomFeatureGenerator()  # create instance
    
    rospy.spin()