import cv2
from aruco_dict import ARUCO_DICT
from pathlib import Path
import numpy as np


_type = "DICT_5X5_50"
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[_type])

Path("./markers/").mkdir(parents=True, exist_ok=True)

for id in range(1,10):
    print("Generating ArUCo tag type '{}' with ID '{}'".format(_type, id))
    tag = np.zeros((300, 300, 1), dtype="uint8")
    cv2.aruco.drawMarker(arucoDict, id, 300, tag, 1)
    cv2.imwrite("./markers/tag_{}.png".format(id), tag)
