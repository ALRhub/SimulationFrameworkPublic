import cv2
import cv2.aruco as aruco
import numpy as np
import transforms3d.quaternions as quat
import yaml
from pyrosenv import rospy
from scipy.spatial.transform import Rotation

from alr_sim.sims.sl_ros.RosSubscriber import RGBImageSubscriber
from alr_sim.sims.sl_ros.RosTfListener import TfListener


class RosLocalizer:
    def __init__(self, calibrared, calib_path) -> None:
        self.rvecs = []
        self.tvecs = []
        self.tvec_calib = []
        self.quat_calib = []
        self.tf_listeners = []
        self.image_subscribers = []
        self.calibrated = calibrared
        self.calib_path = calib_path
        self.tvec_current = None
        self.quat_current = None
        self.updated_position = False

        rospy.init_node("object_localizer", anonymous=True)
        if len(self.calibrated) != 0:
            for cam_id in self.calibrated:
                file_name = self.calib_path + "calibration_cam" + str(cam_id) + ".yaml"
                with open(file_name, "r") as stream:
                    data_dict = yaml.safe_load(stream)
                self.tvec_calib.append(data_dict["t_vec"])
                self.quat_calib.append(data_dict["quaternion"])
                self.tf_listeners.append(
                    TfListener(
                        (
                            "cam_" + str(cam_id) + "/camera_base",
                            "aruco_board_cam" + str(cam_id),
                        )
                    )
                )
                self.image_subscribers.append(
                    RGBImageSubscriber("/aruco_board_cam" + str(cam_id) + "/result")
                )

    def position_updated(self):
        return self.updated_position

    def locate_pos(self, object, cam_list):
        pos, _ = self.get_position(object, cam_list)
        return pos

    def locate_quat(self, object, cam_list):
        _, quat = self.get_position(object, cam_list)
        return quat

    def locate_pos_and_quat(self, object, cam_list):
        pos, quat = self.get_position(object, cam_list)
        return pos, quat

    # Rotate a rotation vector by desired degrees
    def rotate(self, rvec, degr, axis):
        r1 = Rotation.from_rotvec(rvec.flatten())
        r2 = Rotation.from_euler(axis, degr, degrees=True)

        return Rotation.as_rotvec(r1 * r2)

    # Calculate object position with respect to robot base
    def cam2world(
        self,
        cam_pos_in_world,
        cam_quat_in_world,
        obj_pos_in_cam,
        obj_orientation_in_cam,
    ):
        x = cam_quat_in_world[0]
        y = cam_quat_in_world[1]
        z = cam_quat_in_world[2]
        w = cam_quat_in_world[3]
        cam_quat_in_world = np.array([w, x, y, z])
        cam_rot_mat_in_world = quat.quat2mat(cam_quat_in_world)

        rox_x = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        rox_y = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

        # rot_mat_total = cam_rot_mat_in_world @ rox_x @ rox_y
        rot_mat_total = cam_rot_mat_in_world
        obj_pos_world = rot_mat_total @ obj_pos_in_cam + cam_pos_in_world
        obj_rot_world = (
            rot_mat_total @ Rotation.from_rotvec(obj_orientation_in_cam).as_matrix()
        )
        obj_rot_quat = Rotation.from_matrix(
            obj_rot_world
        ).as_quat()  # quaternion of form [x,y,z,w]

        # return quaternion in ROS format
        return obj_pos_world, [
            obj_rot_quat[3],
            obj_rot_quat[0],
            obj_rot_quat[1],
            obj_rot_quat[2],
        ]

    # Calculate position of a object with respect to camera, or if external calibration available to the base of the robot
    def get_position(self, object, camera_list):
        rvec, tvec = [], []

        for cam_id, cam in enumerate(camera_list):
            if self.image_subscribers[cam_id].is_new:
                t, r = self.tf_listeners[cam_id].get_pos_quat()
                # t += object.vec_to_center_ros
                r = [r[1], r[2], r[3], r[0]]
                r = Rotation.from_quat(r).as_rotvec()
                if r is not None and t is not None:
                    rvec.append(r)
                    tvec.append(t)
                else:
                    return None, None
                self.image_subscribers[cam_id].is_new = False

        if len(rvec) != 0 and len(tvec) != 0:
            tvec_mean, rvec_mean = np.mean(tvec, axis=0), np.mean(rvec, axis=0)
            if len(tvec_mean) != 0 and len(self.calibrated) != 0:
                self.tvec_current, self.quat_current = self.cam2world(
                    self.tvec_calib[cam_id],
                    self.quat_calib[cam_id],
                    tvec_mean,
                    rvec_mean,
                )
                self.tvec_current += object.vec_to_center_mujoco
                self.updated_position = True
            return self.tvec_current, self.quat_current

        else:
            self.updated_position = False
            return self.tvec_current, self.quat_current
