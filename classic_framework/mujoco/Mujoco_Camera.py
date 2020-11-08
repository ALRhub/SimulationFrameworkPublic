import numpy as np
import mujoco_py


class Camera:

    def __init__(self, sim, name, fovy, ipd):
        self.sim = sim
        self.width = 1000
        self.height = 1000

        self.name = name

        self.fovy = fovy
        self.ipd = ipd

        # near plane and far plane
        self.near = self.sim.model.vis.map.znear * self.sim.model.stat.extent # as specified in xml file
        self.far = self.sim.model.vis.map.zfar * self.sim.model.stat.extent

        self.fx = (self.width / 2) / (np.tan(self.fovy * np.pi / 180 / 2))
        self.fy = (self.height / 2) / (np.tan(self.fovy * np.pi / 180 / 2))
        self.cx = self.width / 2
        self.cy = self.width / 2

    # def set_fov(self):
    #     self.sim.model.cam_fovy[self.sim.model._camera_name2id[self.name]] = self.fov

    def get_segmentation(self, width=None, height=None, depth=True):
        # self.set_fov()
        if width is None:
            width = self.width
        if height is None:
            height = self.height

        id = self.sim.model._camera_name2id[self.name]
        viewer = mujoco_py.MjRenderContext(self.sim)  # does th same without opening new window
        viewer.render(width, height, id, segmentation=True)
        data = viewer.read_pixels(width, height, depth=False, segmentation=True)[:, :, 1]
        return data

    def get_image(self, width=None, height=None, depth=True):
        if width is None:
            width = self.width
        if height is None:
            height = self.height

        id = self.sim.model._camera_name2id[self.name]

        # viewer = mujoco_py.MjRenderContextOffscreen(self.sim, id)         # leads to open new gflw window :(
        # TODO: put into class constructor. Did not work when tried
        viewer = mujoco_py.MjRenderContext(self.sim)                        # does th same without opening new window
        viewer.render(width, height, id)
        data = viewer.read_pixels(width, height, depth=True)
        rgb_img = data[0]
        if depth:
            depth_img = data[1]
            return rgb_img, depth_img
        else:
            return rgb_img

    def calc_point_cloud(self, width=None, height=None):

        rgb_img, depth_img = self.get_image(width, height)
        z = 2 * self.far * self.near / (self.far + self.near - (self.far - self.near) * (2 * np.array(depth_img) - 1))

        u = np.arange(self.width) - self.cx
        v = np.arange(self.height) - self.cy

        x = (z * u) / self.fx
        y = (z.T * v).T / self.fy

        points = np.stack((x, y, z), axis=-1).reshape((self.width * self.height, 3))
        colors = rgb_img.reshape((self.width * self.height, 3)) / 255

        return points, colors
