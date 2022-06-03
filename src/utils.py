#!/usr/bin/env python3
"""
Utility Functions
"""

from vservo.msg import Point2D, Point2DArray
import numpy as np
import rospy

def conv_Point2DArray_2_nparray(data : Point2DArray) -> np.array:
    if not hasattr(data,'points'):
        return None
    _points = data.points

    if len(_points)==0:
        return None

    out = np.zeros([0,2])               # initialize empty matrix with 2 cols
    for point in _points:
        out=np.append(out,[[point.x, point.y]], axis=0)
    out = np.array(out,dtype=np.int32)
    return out

def conv_nparray_2_Point2DArray(data: np.array) -> Point2DArray:
    out = Point2DArray()
    idx = None
    for idx, row in enumerate(data): # idx is indexing the elements in the loop starting with 0
        [_x,_y] = row
        point = Point2D()
        point.x ,point.y =_x, _y
        out.points.append(point)
    if idx == None:
        out.length=None
    else:
        out.length=idx+1 # final value of idx is last index, add one to get number of iterations on for loop
    return out

def tf_cc(coordinates : np.array) -> np.array:
    """
    Transform Camera Center Coordinates
    """
    if not np.shape(coordinates) == (4,2):
        return np.zeros((0,2))
    width, height = 640, 480 # x=0..640, y=0..480
    return np.subtract(np.repeat([[320,240]],4, axis=0), coordinates)

def conv_pix_2_norm(coordinates: np.array, camera_matrix: np.array) -> np.array:
    """
    Use the camera parameters to calculate coordinates of the normalized plane (x,y) from pixel coordinates (u,v)
    """
    _c_u = camera_matrix[0,2]
    _c_v = camera_matrix[1,2]
    _f_x = camera_matrix[0,0]
    _f_y = camera_matrix[1,1]
    if not np.shape(coordinates) == (4,2):
        rospy.ERROR("Array passed to conv_pix_2_norm does not have shape (4,2)")

    coordinates_new = np.subtract(coordinates, np.repeat([[_c_u, _c_v]],4, axis=0))
    coordinates_new = np.divide(coordinates, np.repeat([[_f_x, _f_y]],4, axis=0))
    return coordinates_new

def conv_norm_2_pix(coordinates: np.array, camera_matrix: np.array) -> np.array:
    """
    Use the camera parameters to calculate pixel coordinates (u,v) from the the normalized coordinates (x,y)
    """
    _c_u = camera_matrix[0,2]
    _c_v = camera_matrix[1,2]
    _f_x = camera_matrix[0,0]
    _f_y = camera_matrix[1,1]
    coordinates_new = np.multiply(coordinates, np.repeat([[_f_x, _f_y]],4, axis=0))
    coordinates_new = np.add(coordinates, np.repeat([[_c_u, _c_v]],4, axis=0))
    coordinates_new = coordinates_new.astype(int) # pixel values are always integers
    return coordinates_new
    