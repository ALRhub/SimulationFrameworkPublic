from typing import Dict

import cv2
import cv2.aruco as aruco

from alr_sim.core.sim_object.sim_object import IntelligentSimObject


class RosObject(IntelligentSimObject):
    def __init__(
        self,
        name: str = None,
        object_dims: list = None,
    ):
        offset_x = object_dims[0] / 2
        offset_y = object_dims[1] / 2
        offset_z = object_dims[2] / 2
        self.vec_to_center_mujoco = [-offset_x, offset_y, offset_z]
        self.vec_to_center_ros = [offset_x, offset_y, -offset_z]

        super().__init__(name, None, None)

    def get_poi(self):
        return [self.name]

    def get_name(self):
        return self.name
