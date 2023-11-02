from logging import warn
from typing import Tuple

import rospy
import tf2_ros


class TfListener:
    def __init__(self, tf_frames: Tuple[str] = None) -> None:
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.tf_frames = tf_frames

    def get_transform_raw(self, tf_frames: Tuple[str] = None):
        if tf_frames is None:
            tf_frames = self.tf_frames

        if tf_frames is None or len(tf_frames) != 2:
            warn("Please specify two desired Transform Frame Names")

        trans = self.buffer.lookup_transform(
            *tf_frames, rospy.Time(), timeout=rospy.Duration(10)
        )
        return trans

    def get_pos_quat(self, tf_frames: Tuple[str] = None):
        trans = self.get_transform_raw(tf_frames).transform
        pos = [trans.translation.x, trans.translation.y, trans.translation.z]
        quat = [trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z]
        return pos, quat


if __name__ == "__main__":
    rospy.init_node("tf2_listener")

    sub = TfListener(("turtle1", "turtle1"))

    rate = rospy.Rate(10.0)
    print(sub.get_pos_quat())
