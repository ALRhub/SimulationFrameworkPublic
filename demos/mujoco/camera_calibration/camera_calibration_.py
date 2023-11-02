import time
from os.path import join, isfile

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.mujoco.MujocoScene import MujocoScene
from alr_sim.sims.mujoco.mj_utils.mujoco_scene_object import MujocoObject
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
from create_chessboard_body import create_chessboard
from cv_calibration import get_intrinsics


class Wakt:
    def __init__(self, delay):
        self.delay = 0
        self.start_time = 0
        self.set(delay)

    def set(self, delay):
        self.delay = delay
        self.start_time = time.time()

    def elapsed(self):
        return time.time() - self.start_time > self.delay


def rotation_matrix(A, B):
    ax = A[0]
    ay = A[1]
    az = A[2]

    bx = B[0]
    by = B[1]
    bz = B[2]

    au = A / (np.sqrt(ax * ax + ay * ay + az * az))
    bu = B / (np.sqrt(bx * bx + by * by + bz * bz))

    rot_mat = np.array(
        [
            [bu[0] * au[0], bu[0] * au[1], bu[0] * au[2]],
            [bu[1] * au[0], bu[1] * au[1], bu[1] * au[2]],
            [bu[2] * au[0], bu[2] * au[1], bu[2] * au[2]],
        ]
    )

    return rot_mat


def get_masks(segmentation_image: np.ndarray, min_area_threshold=50):
    obj_ids = np.unique(segmentation_image)
    masks = {}
    for obj_idx in obj_ids:
        draft_mask = np.zeros(segmentation_image.shape, dtype=np.uint8)
        draft_mask[segmentation_image == obj_idx] = 1
        contours = cv.findContours(
            draft_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )[0]
        if len(contours) == 1:
            contour_idx = 0
        else:
            areas = [
                cv.contourArea(contour)
                for contour in contours
                if cv.contourArea(contour) > min_area_threshold
            ]
            if len(areas) == 0:
                continue

            contour_idx = np.argmax(areas)

        mask = np.zeros(draft_mask.shape, dtype=draft_mask.dtype)
        cv.drawContours(mask, contours, contour_idx, 1, -1)

        masks[str(obj_idx).zfill(2)] = mask

    return masks


def de_projection(cam_intrinsics, cam_pos, cam_rot, coordinates, depth_at_coordinates):
    """transform point coordinates from camera coordinates to world coordinates
    Args:
        cam_pos: position vector of the camera
        cam_rot: rotation matrix of the camera
        coordinates: coordinates in image
        depth_at_coordinates: depth value at coordinates
    Returns:
        real-world coordinates
    """
    # Build Homogenous Transformation Matrix
    z_rot = np.eye(3)
    z_rot[-1, -1] = -1
    cam_rot = np.matmul(cam_rot, z_rot)

    # xyz = np.matmul(np.linalg.inv(cam_intrinsics), depth_at_coordinates * np.insert(coordinates[::-1], 2, 1))
    xyz = np.matmul(
        np.linalg.inv(cam_intrinsics),
        depth_at_coordinates * np.array(tuple(coordinates)[::-1] + (1,)),
    )
    rot_pos = np.append(cam_rot, np.array([cam_pos]).T, axis=1)

    pos = np.matmul(rot_pos, np.append(xyz, 1).reshape(-1, 1))

    return pos.reshape(-1)


if __name__ == "__main__":
    chessboard_path = "./assets/"
    chessboard_images_path = "./assets"

    cam_name = "rgbd"
    duration = 1

    ##### Create the chessboard xml file if missing #####
    if not isfile(join(chessboard_path, "chessboard.xml")):
        create_chessboard(chessboard_path)

    ##### Generate chessboard images from different perspectives #####
    # define objects to load
    table = Box(
        init_pos=[0.6, 0.0, 0.2],
        init_quat=[0, 1, 0, 0],
        name="table0",
        size=[0.4, 0.7, 0.2],
        static=True,
    )

    obj_ = MujocoObject(
        object_name="chessboard",
        pos=[0.45, -0.1, 0.45],
        quat=None,
        obj_path=chessboard_path + "chessboard.xml",
    )

    object_list = [table, obj_]

    sim = SimRepository.get_factory("mujoco_mocap")

    # setup the scene
    mj_Robot = sim.create_robot(gravity_comp=True, num_DoF=7, end_effector="tcp")
    scene: MujocoScene = sim.create_scene(
        mj_Robot, object_list=object_list, render=sim.RenderMode.HUMAN
    )  # if we want to do mocap control
    scene.start()

    mj_Robot.start_logging()
    mj_Robot.set_gripper_width = 0.0

    home_position = mj_Robot.current_c_pos + [-0.15, 0, 0]
    home_orientation = [0, 1, 0, 0]

    print("home_position: ", home_position)
    data = {}
    index = 0

    while index < 15:
        ret = False
        while not ret:
            target_pos = scene.sim.data.body_xpos[
                scene.model.body_name2id("chessboard")
            ] + [0.08, 0.1, 0]
            pos = [
                np.random.normal(target_pos[0] - 0.04, 0.04),
                np.random.normal(target_pos[1], 0.08),
                np.random.uniform(0.45, 0.47, 1),
            ]

            mj_Robot.gotoCartPositionAndQuat(pos, [0, 1, 0, 0], duration=duration)

            # define the direction to target object
            cam_dir_vec = target_pos - scene.sim.data.cam_xpos[1]
            alpha = np.arctan(abs(cam_dir_vec[1]) / abs(cam_dir_vec[2])) * 180 / np.pi
            beta = np.arctan(abs(cam_dir_vec[0]) / abs(cam_dir_vec[2])) * 180 / np.pi
            v = -np.sign(cam_dir_vec[1])
            u = -np.sign(cam_dir_vec[0])
            euler_angles = [v * (180 - alpha), u * beta, 0]

            angle_rot = R.from_euler("xyz", euler_angles, True)
            angle_quat = R.from_matrix(angle_rot.as_matrix()).as_quat()
            angle_quat = np.insert(angle_quat[:3], 0, angle_quat[3])
            timer = Wakt(duration)
            while not timer.elapsed():
                mj_Robot.gotoCartPositionAndQuat(pos, angle_quat, duration=duration / 2)

            img = scene.inhand_cam.get_image(depth=False)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            ret, _ = cv.findChessboardCorners(gray, (7, 6), None)

        plt.imshow(img), plt.show()
        cv.imwrite(
            join(chessboard_images_path, str(index).zfill(2) + ".png"),
            cv.cvtColor(img, cv.COLOR_RGB2BGR),
        )
        index += 1

    ##### Compute the intrinsics of the camera ######
    print(get_intrinsics(chessboard_images_path))
