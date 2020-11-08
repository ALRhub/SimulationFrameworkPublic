"""
This module contains the camera base class as well as a class for the robots in hand camera and cage cam.
"""
import cv2
import pybullet as p
import math
import numpy as np


class Camera:
    """
    Camera base class.
    """

    def __init__(self):
        # TODO Load from yaml for different camera configurations
        self.intrinsics = np.array(
            [[596.6278076171875, 0.0, 311.98663330078125, 0.0],
             [0.0, 596.6278076171875, 236.76170349121094, 0.0],
             [0.0, 0.0, 1.0, 0.0]])

        self.width = 640
        self.height = 480
        self.fx = self.intrinsics[0, 0]  # focal length
        self.fy = self.intrinsics[1, 1]
        self.cx = self.intrinsics[0, 2]  # principal point
        self.cy = self.intrinsics[1, 2]

        self.fov = 2 * math.atan(self.height / 2 / self.fy) * 180 / math.pi  # field of view angle
        # self.fov = 2*math.atan(self.pixel_width/2/self.intrinsics[0, 0])*180/math.pi  # field of view angle
        self.near = 0.01
        self.far = 10
        self.init_camera_vector = (0, 0, 1)  # z axis
        self.init_up_vector = (0, -1, 0)
        self.plt = None
        self.fig = None
        self.ax = None
        self.lookat = None

        # cf https://blog.noctua-software.com/opencv-opengl-projection-matrix.html
        # except at (1, 1) where he put a wrong minus
        self.projection_matrix = (2.0 * self.fx / self.width, 0.0, 0.0, 0.0,
                                  0.0, 2.0 * self.fy / self.height, 0.0, 0.0,
                                  1.0 - 2.0 * self.cx / self.width, 2.0 * self.cy / self.height - 1.0,
                                  (self.far + self.near) / (self.near - self.far), -1.0,
                                  0.0, 0.0, 2.0 * self.far * self.near / (self.near - self.far), 0.0)
        self.eye_in_hand_camera_projection_matrix = self.projection_matrix

    def getPosRot(self, id, client_id):
        pos = None
        rot = None
        rot_matrix_matrix = None
        return pos, rot, rot_matrix_matrix

    def get_view_matrix(self, id, client_id):
        pos, rot, rot_matrix = self.getPosRot(id, client_id)

        camera_vector = rot_matrix.dot(self.init_camera_vector)
        up_vector = rot_matrix.dot(self.init_up_vector)

        camera_eye_pos = np.array(pos)
        camera_target_position = camera_eye_pos + 0.2 * camera_vector
        self.lookat = camera_target_position

        view_matrix = p.computeViewMatrix(camera_eye_pos,
                                          camera_target_position,
                                          up_vector)

        return view_matrix

    def calc_point_cloud(self, id, client_id, with_noise=False):
        """
        returns the 3d world coordinates and the corresponding rgb values of inHandCamera.
        :param client_id: id of the client simulating in
        :param id: if inHandCam -> equal to robot_id in the physics client
                   if CageCam -> equal to id of the cam in the physics client
        :param plot_points: whether the points shall be plot in matplotlib (very slow)
        :return: points: np array (nb_pointsx3)
                 colors: np array (nb_pointsx4)
        """
        rgb_img, z, _ = self.get_image(id, client_id, with_noise=with_noise)

        u = np.arange(self.width) - self.cx
        v = np.arange(self.height) - self.cy

        x = (z * u) / self.fx
        y = (z.T * v).T / self.fy

        points = np.stack((x, y, z), axis=-1).reshape((self.width * self.height, 3))
        colors = rgb_img.reshape((self.width * self.height, 4))

        # if plot_points:
        #     if self.plt is None:
        #         import matplotlib.pyplot as plt
        #         self.plt = plt
        #         self.fig = plt.figure()
        #         self.ax = self.fig.add_subplot(111, projection='3d')
        #
        #     self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors)
        #     self.plt.show()
        #     self.plt.pause(0.01)

        # returns all points in points (nb_points x 3) and the correspoinding rgb colors (nb_points x 4) [r,g,b,1]
        return points, colors

    def clear_points_in_plot(self):
        """
        Clear the current axis of the plot.

        :return: no return value
        """
        self.plt.cla()

    def remove_created_bodies(self, list_ids, n_bodies):
        for i in range(n_bodies):
            p.removeBody(list_ids[i])
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    def get_image(self, cam_id, client_id, with_noise=False, shadow=True):
        """
        Returns the rgb, depth and segamsks as np array of the inHand camera.

        :param: robot_id: robot id which carries the inhand camera
        :param: client_id: id of the client simulating in
        :return: np array of rgb, depth, segmask images
        """
        view_matrix = self.get_view_matrix(cam_id, client_id)

        if shadow:
            _, _, rgb_img, depth_img, seg_img = p.getCameraImage(
                self.width, self.height,
                view_matrix, self.projection_matrix,
                shadow=0, lightDirection=[1, 1, 1],
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
                physicsClientId=client_id)
        else:
            _, _, rgb_img, depth_img, seg_img = p.getCameraImage(
                self.width, self.height,
                view_matrix, self.projection_matrix,
                shadow=1, lightDirection=[1, 1, 1],
                renderer=p.ER_TINY_RENDERER,
                physicsClientId=client_id)

        rgb_img = np.array(rgb_img) / 255.

        # do not use this
        # rgb_img = np.reshape(rgb_img, (self.height, self.width, 4))
        # depth_img = np.reshape(depth_img, (self.height, self.width))
        # seg_img = np.reshape(seg_img, (self.height, self.width))

        # From normalized to actual depth, as described here:
        # http://web.archive.org/web/20130416194336/http://olivers.posterous.com/linear-depth-in-glsl-for-real
        z = 2 * self.far * self.near / (self.far + self.near - (self.far - self.near) * (2 * np.array(depth_img) - 1))

        if with_noise:
            # Gaussian noise (scale with power of 2 of the real depth + offset)
            depth_img = depth_img + (0.0001 * np.power(z - 0.5, 2) + 0.0004) * np.random.rand(self.height, self.width)

            # Quantization (scale with the inverse depth)
            depth_img = ((40000 * depth_img).astype(int) / 40000.).astype(np.float32)

            # From normalized to actual depth, with noise
            z = 2 * self.far * self.near / (self.far + self.near - (self.far - self.near) * (2 * depth_img - 1))

            # Final smoothing with a bilateral filter
            # args: kernel size, sigma in "color" space, sigma in coordinate space
            z = cv2.bilateralFilter(z, 5, 0.1, 5)

        return rgb_img, z, seg_img


class InHandCamera(Camera):
    """
    In hand camera of the robot. Extends camera base class.
    """

    def __init__(self):
        Camera.__init__(self)

    def getPosRot(self, cam_id, client_id):
        pos, rot, _, _, _, _ = p.getLinkState(cam_id, linkIndex=20,
                                              computeForwardKinematics=True,
                                              physicsClientId=client_id)
        rot_matrix = p.getMatrixFromQuaternion(rot)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        return pos, rot, rot_matrix


class CageCamera(Camera):
    """
    Cage camera. Extends the camera base class.
    """

    def __init__(self):
        Camera.__init__(self)

    def getPosRot(self, id, client_id):
        pos, rot = p.getBasePositionAndOrientation(id, physicsClientId=client_id)
        rot_matrix = p.getMatrixFromQuaternion(rot)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        return pos, rot, rot_matrix
