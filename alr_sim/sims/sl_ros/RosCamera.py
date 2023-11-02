import logging
import os
import sys
from typing import Optional, Tuple

import numpy as np
import rospy
import yaml

from alr_sim.core.Camera import Camera
from alr_sim.sims.sl_ros.InHandTransformer import InHandTransformer
from alr_sim.sims.sl_ros.RosSubscriber import (
    CameraInfoSubscriber,
    DepthImageSubscriber,
    IrImageSubscriber,
    PointCloudSubscriber,
    RGBImageSubscriber,
    SegmentationImageSubscriber,
)
from alr_sim.sims.sl_ros.RosTfListener import TfListener


class RosCamera(Camera):
    def __init__(
        self,
        name: str,
        rgb_topic: str,
        depth_topic: str,
        bgr_order: bool = False,
        pcl_topic: str = None,
        ir_topic: str = None,
        seg_topic: str = None,
        cam_params_topic: str = None,
        transform_frames: Tuple[str] = None,
        inhand_transformer: InHandTransformer = None,
        init_pos=None,
        init_quat=None,
        near: float = 0.01,
        far: float = 10.0,
        fov: int = 45,
        intrinsics: np.ndarray = None,
        *args,
        **kwargs
    ):
        super(RosCamera, self).__init__(
            name, init_pos=init_pos, init_quat=init_quat, near=near, far=far, fov=fov
        )

        if intrinsics is not None:
            self.fx = intrinsics[0, 0]
            self.fy = intrinsics[1, 1]
            self.cx = intrinsics[0, 2]
            self.cy = intrinsics[1, 2]
            self.fov = 2 * np.arctan(self.height / 2.0 / self.fy) * 180 / np.pi
        else:
            intrinsics = np.array(
                [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]]
            )
        self._intrinsics = intrinsics

        try:
            rospy.init_node("cam_listener", anonymous=True)
        except rospy.exceptions.ROSException as e:
            logging.getLogger(__name__).info(
                "Node has already been initialized, do nothing"
            )

        self.rgb_listener = RGBImageSubscriber(rgb_topic)
        self.depth_listener = DepthImageSubscriber(depth_topic)
        if seg_topic is not None:
            self.seg_listener = SegmentationImageSubscriber(seg_topic)
        else:
            self.seg_listener = None

        self.cam_params_listener = None
        if cam_params_topic is not None:
            self.cam_params_listener = CameraInfoSubscriber(cam_params_topic)

        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.bgr_order = bgr_order
        self.pcl_topic = pcl_topic

        self.pcl_listener = None
        if pcl_topic is not None:
            self.pcl_listener = PointCloudSubscriber(pcl_topic)

        self.ir_listener = None
        if ir_topic is not None:
            self.ir_listener = IrImageSubscriber(ir_topic)

        self.transform_listener = None
        self.transform_frames = None
        if transform_frames is not None:
            self.transform_listener = TfListener(transform_frames)

        self.inhand_transformer = inhand_transformer

    def _get_img_data(
        self, width: int = None, height: int = None, depth: bool = True, *args, **kwargs
    ):
        rgb_img = self.rgb_listener.get_image_data(self.bgr_order)

        if depth:
            depth_img = self.depth_listener.get_image_data()
            return rgb_img, depth_img

        return rgb_img

    def get_ir_image(self):
        return self.ir_listener.get_image_data()

    def get_cam_params(self):
        K, D = self.cam_params_listener.get_cam_params()
        self._intrinsics = K
        self.dist_params = D

        return K, D

    def calc_point_cloud(
        self, width: int = None, height: int = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        if self.pcl_listener is None:
            return super().calc_point_cloud(denormalize=False)

        else:
            return self.pcl_listener.get_point_cloud_data()

    def get_cart_pos_quat(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.inhand_transformer is not None:
            return self.inhand_transformer.get_pos_quat()

        if self.transform_listener is None:
            return self.init_pos, self.init_quat

        pos, quat = self.transform_listener.get_pos_quat()
        return np.array(pos), np.array(quat)

    def get_segmentation(self, *args, **kwargs) -> np.ndarray:
        if self.seg_listener is None:
            return np.zeros((self.height, self.width))

        seg_img = self.seg_listener.get_image_data()

        return seg_img

    def save_yaml(self, dir):
        data = {
            "name": self.name,
            "rgb_topic": self.rgb_topic,
            "depth_toic": self.depth_topic,
            "bgr_order": self.bgr_order,
            "pcl_topic": self.pcl_topic,
        }

        os.makedirs(dir, exist_ok=True)
        with open(os.path.join(dir, "{}.yml".format(self.name)), "w") as f:
            yaml.dump(data, f, default_flow_style=False)

    def set_cam_params(
        self,
        width: Optional[int] = None,
        height: Optional[int] = None,
        near: Optional[float] = None,
        far: Optional[float] = None,
        fovy: Optional[int] = None,
    ):
        self.width = width
        self.height = height
        self.near = near
        self.far = far
        self.fovy = fovy
        pass

    @property
    def intrinsics(self):
        K, D = self.get_cam_params()
        return K.reshape(3, 3)

    @intrinsics.setter
    def intrinsics(self, value):
        self._intrinsics = value
        pass

    @staticmethod
    def load_yaml(f_path):
        with open(f_path, "r") as f:
            d = yaml.load(f)
            return RosCamera(**d)
