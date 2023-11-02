import numpy as np
import pinocchio
import ros_numpy
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, JointState, PointCloud2

from alr_sim.core import Camera, RobotBase
from alr_sim.utils.geometric_transformation import mat2quat, quat2mat, quat_rot_vec
from alr_sim.utils.point_clouds import rgb_array_to_uint32, rgb_float_to_int
from alr_sim.utils.sim_path import sim_framework_path


class TFBroadcaster:
    def __init__(self, robot: RobotBase, camera: Camera, tick: int = 100):
        self.tick = tick
        self._robot = robot
        self._cam = camera
        obj_urdf = sim_framework_path("./models/pybullet/robots/panda_arm_hand.urdf")
        self.pinocchio_robot = pinocchio.buildModelFromUrdf(obj_urdf)
        self.br = tf2_ros.TransformBroadcaster()
        self._frame_ids = list(self.pinocchio_robot.names)
        self._frame_ids[0] = "world"
        # self._frame_ids[0] = "map"

    def run_once(self):
        time_stamp = self._robot.time_stamp
        secs = int(time_stamp // 1)
        nsecs = int(time_stamp * 1000 % 1000)

        # Robot state transform
        data = self.pinocchio_robot.createData()
        q = np.zeros(len(self._frame_ids) - 1)
        q[:-2] = self._robot.current_j_pos  # todo fingers
        pinocchio.forwardKinematics(self.pinocchio_robot, data, q)
        translation = [data.liMi[i].translation for i in range(len(self._frame_ids))]
        rotation = [data.liMi[i].rotation for i in range(len(self._frame_ids))]

        for i in range(0, len(self._frame_ids) - 1):
            tf_msg = TransformStamped()
            tf_msg.header.stamp.secs = secs
            tf_msg.header.stamp.nsecs = nsecs

            tf_msg.header.frame_id = self._frame_ids[i]
            tf_msg.child_frame_id = self._frame_ids[i + 1]
            tf_msg.transform.translation.x = translation[i + 1][0]
            tf_msg.transform.translation.y = translation[i + 1][1]
            tf_msg.transform.translation.z = translation[i + 1][2]
            q = mat2quat(rotation[i + 1])
            tf_msg.transform.rotation.x = q[1]
            tf_msg.transform.rotation.y = q[2]
            tf_msg.transform.rotation.z = q[3]
            tf_msg.transform.rotation.w = q[0]
            self.br.sendTransform(tf_msg)

        # Camera state transform
        cam_pos, cam_quat = self._cam.get_cart_pos_quat()
        cam_tf_msg = TransformStamped()
        cam_tf_msg.header.stamp.secs = secs
        cam_tf_msg.header.stamp.nsecs = nsecs
        cam_tf_msg.header.frame_id = self._frame_ids[0]
        cam_tf_msg.child_frame_id = self._cam.name
        cam_tf_msg.transform.translation.x = cam_pos[0]
        cam_tf_msg.transform.translation.y = cam_pos[1]
        cam_tf_msg.transform.translation.z = cam_pos[2]
        cam_tf_msg.transform.rotation.w = cam_quat[0]
        cam_tf_msg.transform.rotation.x = cam_quat[1]
        cam_tf_msg.transform.rotation.y = cam_quat[2]
        cam_tf_msg.transform.rotation.z = cam_quat[3]
        self.br.sendTransform(cam_tf_msg)


class RosPublisher:
    def __init__(self, topic: str, msg_type, tick: int = 100):
        self.pub = rospy.Publisher(topic, msg_type, queue_size=10)
        self.tick = tick
        self._msg = None

    def set_msg(self, msg):
        self._msg = msg

    def _get_msg(self):
        return self._msg

    def run_once(self):
        msg = self._get_msg()
        if msg is not None:
            self.pub.publish(msg)


class JointStatePublisher(RosPublisher):
    def __init__(self, robot: RobotBase, tick: int = 10):
        super(JointStatePublisher, self).__init__("/joint_state", JointState, tick)
        self._robot = robot

    def _get_msg(self):
        msg = JointState()
        time_stamp = self._robot.time_stamp
        msg.header.stamp.secs = int(time_stamp // 1)
        msg.header.stamp.nsecs = int(time_stamp * 1000 % 1000)
        msg.name = np.asarray([f"joint{i}" for i in range(7)])
        msg.position = self._robot.current_j_pos
        msg.velocity = self._robot.current_j_vel
        msg.effort = self._robot.current_load
        return msg


class CartesianStatePublisher(RosPublisher):
    def __init__(self, robot: RobotBase, tick: int = 10):
        super(CartesianStatePublisher, self).__init__("/cart_state", PoseStamped, tick)
        self._robot = robot

    def _get_msg(self):
        msg = PoseStamped()
        time_stamp = self._robot.time_stamp
        msg.header.stamp.secs = int(time_stamp // 1)
        msg.header.stamp.nsecs = int(time_stamp * 1000 % 1000)
        msg.pose.position.x = self._robot.current_c_pos[0]
        msg.pose.position.y = self._robot.current_c_pos[1]
        msg.pose.position.z = self._robot.current_c_pos[2]
        msg.pose.orientation.x = self._robot.current_c_quat[0]
        msg.pose.orientation.y = self._robot.current_c_quat[1]
        msg.pose.orientation.z = self._robot.current_c_quat[2]
        msg.pose.orientation.w = self._robot.current_c_quat[3]
        return msg


class CartesianVelocityPublisher(RosPublisher):
    def __init__(self, robot: RobotBase, tick: int = 10):
        super(CartesianVelocityPublisher, self).__init__(
            "/cart_velocity", PoseStamped, tick
        )
        self._robot = robot

    def _get_msg(self):
        msg = PoseStamped()
        time_stamp = self._robot.time_stamp
        msg.header.stamp.secs = int(time_stamp // 1)
        msg.header.stamp.nsecs = int(time_stamp * 1000 % 1000)
        msg.pose.position = self._robot.current_c_vel
        msg.pose.quaternion = self._robot.current_c_quat_vel
        return msg


class GripperState(RosPublisher):
    def __init__(self, robot: RobotBase, tick: int = 10):
        super(GripperState, self).__init__("/gripper_state", JointState, tick)
        self._robot = robot

    def _get_msg(self):
        msg = JointState()
        time_stamp = self._robot.time_stamp
        msg.header.stamp.secs = int(time_stamp // 1)
        msg.header.stamp.nsecs = int(time_stamp * 1000 % 1000)
        raise NotImplementedError


class PclPublisher(RosPublisher):
    def __init__(self, robot: RobotBase, camera: Camera, tick: int = 100):
        super(PclPublisher, self).__init__(camera.name + "/pcl", PointCloud2, tick)
        self._robot = robot
        self._cam = camera

    def _get_msg(self):
        cam_pos, cam_quat = self._cam.get_cart_pos_quat()
        xyz, rgb = self._cam.calc_point_cloud()
        data = np.zeros(
            xyz.shape[0],
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("rgb", np.uint32),
            ],
        )
        data["x"] = xyz[:, 0]
        data["y"] = xyz[:, 1]
        data["z"] = xyz[:, 2]

        rgb = rgb_float_to_int(rgb)
        data["rgb"] = rgb_array_to_uint32(rgb)

        msg = ros_numpy.msgify(PointCloud2, data)

        time_stamp = self._robot.time_stamp
        msg.header.stamp.secs = int(time_stamp // 1)
        msg.header.stamp.nsecs = int(time_stamp * 1000 % 1000)
        msg.header.frame_id = self._cam.name

        return msg


class PclFloatPublisher(RosPublisher):
    def __init__(self, camera: Camera, tick: int = 100):
        raise NotImplementedError
        super(PclFloatPublisher, self).__init__(
            camera.name + "/pcl_float", numpy_msg(Floats), tick
        )
        self.cam = camera

    def _get_msg(self):
        xyz, rgb = self.cam.calc_point_cloud()
        msg = np.stack([xyz, rgb])
        return msg


class ImgPublisher(RosPublisher):
    def __init__(self, robot: RobotBase, camera: Camera, tick: int = 100):
        super(ImgPublisher, self).__init__(camera.name + "/rgb", Image, tick)
        self._cam = camera
        self._robot = robot

    def _get_msg(self):
        img, depth = self._cam.get_image()
        msg = ros_numpy.msgify(Image, img, "rgb8")

        time_stamp = self._robot.time_stamp
        msg.header.stamp.secs = int(time_stamp // 1)
        msg.header.stamp.nsecs = int(time_stamp * 1000 % 1000)
        msg.header.frame_id = self._cam.name
        return msg


class DepthImgPublisher(RosPublisher):
    def __init__(self, robot: RobotBase, camera: Camera, tick: int = 100):
        super(DepthImgPublisher, self).__init__(camera.name + "/depth", Image, tick)
        self._cam = camera
        self._robot = robot

    def _get_msg(self):
        img, depth = self._cam.get_image()
        msg = ros_numpy.msgify(Image, depth, "32FC1")

        time_stamp = self._robot.time_stamp
        msg.header.stamp.secs = int(time_stamp // 1)
        msg.header.stamp.nsecs = int(time_stamp * 1000 % 1000)
        msg.header.frame_id = self._cam.name
        return msg
