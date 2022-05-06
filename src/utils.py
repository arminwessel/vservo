#!/usr/bin/env python3
"""
Utility Functions
"""

from vservo.msg import Point2D, Point2DArray
import numpy as np
import rospy

def conv_Point2DArray_2_nparray(data : Point2DArray) -> np.array:
    if not hasattr(data,'points'):
        return
    _points = data.points

    out = np.zeros([0,2])               # initialize empty matrix with 2 cols
    for point in _points:
        out=np.append(out,[[point.x, point.y]], axis=0)
    out = np.array(out,dtype=np.int32)
    return out

def conv_nparray_2_Point2DArray(data: np.array) -> Point2DArray:
    out = Point2DArray()
    for idx, row in enumerate(data): # idx is indexing the elements in the loop starting with 0
        [_x,_y] = row
        point = Point2D()
        point.x ,point.y =_x, _y
        out.points.append(point)
    out.length=idx+1 # final value of idx is last index, add one to get number of iterations on for loop
    return out