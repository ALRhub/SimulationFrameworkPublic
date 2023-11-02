import logging
import sys
import time

import numpy as np
import ros_numpy
import rospy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2


class RosSubscriber:
    def __init__(self, ros_topic: str, msg_type):

        if ros_topic not in [
            topic_name for (topic_name, topic_type) in rospy.get_published_topics()
        ]:
            raise NameError("Could not find topic {}".format(ros_topic))

        rospy.Subscriber(ros_topic, msg_type, self._save_msg)
        self._data = None
        self._seq = -1
        self._stamp = -1
        self.is_new = True

    def _save_msg(self, data):
        self._data = data
        self._seq = data.header.seq
        self._stamp = data.header.stamp
        self.is_new = True

    def get_data_raw(self):
        while not self.is_new:
            logging.getLogger(__name__).info("Listening for data...")
            time.sleep(0.1)
        return self._stamp, self._seq, self._data

    def get_numpy_data(self) -> np.ndarray:
        _, _, data = self.get_data_raw()
        return ros_numpy.numpify(data)


class RGBImageSubscriber(RosSubscriber):
    def __init__(self, topic=None):
        super().__init__(topic, Image)

    def get_image_data(self, bgr_order=False):
        rgb_img = self.get_numpy_data()
        rgb_img = rgb_img[..., :3]

        if bgr_order:
            rgb_img = np.flip(rgb_img, axis=-1)

        return rgb_img


class DepthImageSubscriber(RosSubscriber):
    def __init__(self, topic=None):
        super().__init__(topic, Image)

    def get_image_data(self):
        depth_img = ros_numpy.numpify(self._data)

        if "16" in self._data.encoding:
            depth_img = depth_img / 1000.0

        return depth_img


class SegmentationImageSubscriber(RosSubscriber):
    def __init__(self, topic=None):
        super().__init__(topic, Image)

    def get_image_data(self):
        seg_img = ros_numpy.numpify(self._data)

        return seg_img


class PointCloudSubscriber(RosSubscriber):
    def __init__(self, topic=None):
        super().__init__(topic, PointCloud2)

    def get_point_cloud_data(self):
        pc = self.get_numpy_data()

        xyz = np.zeros((pc.shape[0], 3))
        xyz[:, 0] = pc["x"]
        xyz[:, 1] = pc["y"]
        xyz[:, 2] = pc["z"]

        if "r" not in pc.dtype.names:
            pc = ros_numpy.point_cloud2.split_rgb_field(pc)
        rgb = np.zeros((pc.shape[0], 3))
        rgb[:, 0] = pc["r"]
        rgb[:, 1] = pc["g"]
        rgb[:, 2] = pc["b"]
        return xyz, rgb


class IrImageSubscriber(RosSubscriber):
    def __init__(self, topic=None):
        super().__init__(topic, Image)

    def get_image_data(self):
        return ros_numpy.numpify(self._data)


class JointStateSubscriber(RosSubscriber):
    def _init_(self, topic=None):
        super().__init__(topic, JointState)

    def get_joint_data(self):
        position = self._data.position
        velocity = self._data.velocity
        effort = self._data.effort

        return position, velocity, effort


class CameraInfoSubscriber(RosSubscriber):
    def __init__(self, topic=None):
        super().__init__(topic, CameraInfo)

    def get_cam_params(self):
        # K and D are the intrisic camera parameters
        K = np.asarray(self._data.K).reshape((3, 3))
        D = np.asarray(self._data.D)

        # Return camera matrix K (3x3) and distortion coefficients D (5x1)
        return K, D


"""
class CartesiaStateSubscriber(RosSubscriber):
    def __init__(self, ros_topic):
        super().__init__(ros_topic, PoseStamped)

    def get_cartesian_state(self):
        pos_x = self._data.pose.position.x
        pos_y = self._data.pose.position.y
        pos_z = self._data.pose.position.z
        orientation_x = self._data.pose.orientation.x
        orientation_y = self._data.pose.orientation.y
        orientation_z = self._data.pose.orientation.z
        orientation_w = self._data.pose.orientation.w

        position = [pos_x, pos_y, pos_z]
        orientation = [orientation_x, orientation_y, orientation_z, orientation_w]

        return position, orientation


class CartesianVelocitySubscriber(RosSubscriber):
    def __init__(self, ros_topic):
        super().__init__(ros_topic, PoseStamped)

    def get_cartesian_velocity(self):
        pos = self._data.pose.position
        quat = self._data.pose.quaternion

        return pos, quat
"""


class GripperStateSubscriber(RosSubscriber):
    def _init(self, ros_topic):
        super().__init__(ros_topic, JointState)

    def get_gripper_state(self):
        raise NotImplementedError
